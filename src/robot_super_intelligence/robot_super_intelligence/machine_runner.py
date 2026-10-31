"""Callback-driven state sequencing within the manager's existing run."""

from .outcomes import Outcome


class MachineRunner:
    def __init__(self, node, machine, dispatch, feedback):
        self.node = node
        self.machine = machine
        self.dispatch = dispatch
        self.feedback = feedback
        self.on_complete = None
        self.done = False
        self.generation = 0

    def start(self, on_complete):
        self.on_complete = on_complete
        self.schedule()

    def schedule(self):
        # Queue transitions so immediately completed states cannot recurse.
        self.node.executor.create_task(self.enter)

    def enter(self):
        if self.done:
            return
        self.generation += 1
        token = self.generation
        try:
            self.feedback()
            self.machine.state.start(
                self.dispatch, lambda outcome: self.state_completed(token, outcome))
        except Exception as error:
            self.node.get_logger().error(f'State failed: {error}')
            self.stop(Outcome.FAILED)

    def state_completed(self, token, outcome):
        if not self.done and token == self.generation:
            self.node.executor.create_task(self.completed, token, outcome)

    def completed(self, token, outcome):
        if self.done or token != self.generation:
            return
        # Consume this state's completion once, even if it calls back twice.
        self.generation += 1
        try:
            self.machine.advance(outcome)
            self.feedback()
            if self.machine.done:
                self.finish(self.machine.result)
            else:
                self.schedule()
        except Exception as error:
            self.node.get_logger().error(f'Transition failed: {error}')
            self.stop(Outcome.FAILED)

    def finish(self, outcome):
        if self.done:
            return
        self.done = True
        self.on_complete(outcome)

    def stop(self, outcome):
        if self.done:
            return
        self.generation += 1
        try:
            self.machine.end(outcome)
            self.machine.advance(Outcome.RUN_END)
            self.feedback()
        except Exception as error:
            self.node.get_logger().error(f'State cleanup failed: {error}')
        finally:
            self.finish(outcome)
