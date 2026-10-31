"""Future-driven state sequencing within the manager's existing run."""

from rclpy.task import Future

from .machine import Machine
from .outcomes import Outcome


class FullAuto:
    def __init__(self, node, limit, dispatch, feedback, machine_factory=Machine):
        self.node = node
        self.machine = machine_factory(limit)
        self.dispatch = dispatch
        self.feedback = feedback
        self.future = Future(executor=node.executor)
        self.generation = 0

    def start(self):
        self.schedule()
        return self.future

    def schedule(self):
        # Queue each transition; immediately completed states must not recurse.
        self.node.executor.create_task(self.enter)

    def enter(self):
        if self.future.done():
            return
        token = self.generation
        try:
            self.feedback()
            future = self.machine.state.start(self.dispatch)
            future.add_done_callback(lambda done: self.completed(token, done))
        except Exception as error:
            self.node.get_logger().error(f'State failed: {error}')
            self.stop(Outcome.FAILED)

    def completed(self, token, future):
        if self.future.done() or token != self.generation:
            return
        try:
            self.machine.advance(future.result())
            self.feedback()
            if self.machine.done:
                self.future.set_result(self.machine.result)
            else:
                self.schedule()
        except Exception as error:
            self.node.get_logger().error(f'Transition failed: {error}')
            self.stop(Outcome.FAILED)

    def stop(self, outcome):
        if self.future.done():
            return
        self.generation += 1
        try:
            self.machine.end(outcome)
            self.machine.advance(Outcome.RUN_END)
        finally:
            self.feedback()
            self.future.set_result(outcome)
