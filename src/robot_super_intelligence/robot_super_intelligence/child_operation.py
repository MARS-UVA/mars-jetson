"""One child action at a time, including routing and cancellation races."""

from action_msgs.msg import GoalStatus

from .outcomes import Outcome


class ChildOperation:
    def __init__(self, node, control, operations, on_feedback, on_failure, on_idle):
        self.node = node
        self.control = control
        self.operations = operations
        self.on_feedback = on_feedback
        self.on_failure = on_failure
        self.on_idle = on_idle
        self.active = False
        self.goal = None
        self.reason = ''
        self.cancel_sent = False
        self.result_retry = False

    def start(self, request, on_complete):
        """Report an outcome after the child is terminal and its source released."""
        if self.active:
            on_complete('CHILD_BUSY')
            return
        self.active = True
        self.on_complete = on_complete
        if self.reason:
            self.finish(self.reason)
            return
        state = self.operations.get(request.operation)
        if state is None or not state.client.server_is_ready():
            self.finish('SERVER_UNAVAILABLE')
        elif not state.valid(request):
            self.finish('INVALID_GOAL')
        else:
            self.control.select(state.source,
                                lambda error: self.selected(state, request, error))

    def selected(self, state, request, error):
        if not self.active:
            return
        if error is not None or self.reason:
            self.finish(self.reason or Outcome.FAILED.value)
            return
        try:
            self.on_feedback(state.source)

            def feedback(message):
                if self.active:
                    self.on_feedback(getattr(message.feedback, 'phase', state.source))

            state.send_goal(request, feedback, self.accepted)
        except Exception as error:
            self.node.get_logger().error(f'Cannot send child goal: {error}')
            self.finish(Outcome.FAILED.value)

    def accepted(self, future):
        try:
            child = future.result()
        except Exception as error:
            if self.active:
                self.node.get_logger().error(f'Child goal response failed: {error}')
                self.finish(Outcome.FAILED.value)
            return
        if not self.active:
            if child.accepted:
                child.cancel_goal_async()
            return
        if not child.accepted:
            self.finish('GOAL_REJECTED')
            return
        self.goal = child
        self.cancel_sent = False
        self.result_retry = False
        # Cancellation may have arrived while acceptance was pending.
        self.request_result(child)
        if self.reason:
            self.cancel(self.reason)

    def cancel(self, reason):
        self.reason = reason
        if self.goal is not None and not self.cancel_sent:
            self.cancel_sent = True
            self.goal.cancel_goal_async().add_done_callback(self.cancel_response)

    def cancel_response(self, future):
        try:
            if not future.result().goals_canceling:
                self.node.get_logger().warn(
                    'Child cancellation rejected or already terminal; waiting for result')
        except Exception as error:
            self.node.get_logger().error(
                f'Child cancellation failed: {error}; command lock remains active')

    def request_result(self, child):
        try:
            child.get_result_async().add_done_callback(
                lambda future: self.result(child, future))
        except Exception as error:
            self.result_error(child, error)

    def result_error(self, child, error):
        self.node.get_logger().error(f'Child result failed: {error}')
        self.on_failure(Outcome.FAILED.value)
        # An unknown terminal state keeps ownership reserved, even after retry.
        if not self.result_retry:
            self.result_retry = True
            self.request_result(child)

    def result(self, child, future):
        if not self.active or self.goal is not child:
            return
        try:
            result = future.result()
        except Exception as error:
            self.result_error(child, error)
            return
        self.goal = None
        succeeded = (result.status == GoalStatus.STATUS_SUCCEEDED
                     and getattr(result.result, 'success', True))
        self.finish(self.reason or ('SUCCESS' if succeeded else 'CHILD_FAILED'))

    def finish(self, outcome):
        def stopped(error):
            if not self.active:
                return
            self.active = False
            if error is not None:
                self.node.get_logger().error(f'Cannot release child source: {error}')
                self.on_failure(Outcome.FAILED.value)
            try:
                self.on_complete(self.reason or outcome)
            except Exception as error:
                self.node.get_logger().error(f'Child completion callback failed: {error}')
                self.on_failure(Outcome.FAILED.value)
            self.on_idle()

        self.control.select('', stopped)
