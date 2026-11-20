"""AprilTags helper: gate call results, client-side timeout and cancel."""

from types import SimpleNamespace
from unittest.mock import Mock

from robot_super_intelligence.apriltags import AprilTags


class FakeNode:
    def __init__(self):
        self.timers, self.destroyed, self.futures = [], [], []
        node = self

        class Client:
            def call_async(self, request):
                future = SimpleNamespace(callbacks=[])
                future.add_done_callback = future.callbacks.append
                node.futures.append(future)
                return future

            def service_is_ready(self):
                return True

        self.client = Client()

    def create_client(self, *args):
        return self.client

    def create_timer(self, period, callback, clock=None):
        timer = SimpleNamespace(period=period, fire=callback)
        self.timers.append(timer)
        return timer

    def destroy_timer(self, timer):
        self.destroyed.append(timer)

    def get_logger(self):
        return Mock()


def answer(future, success=None, error=None):
    def result():
        if error:
            raise error
        return SimpleNamespace(success=success)
    future.result = result
    for callback in future.callbacks:
        callback(future)


def helper():
    node = FakeNode()
    return node, AprilTags(node)


def test_reports_the_gate_answer_and_clears_its_timer():
    for success in (True, False):
        node, tags = helper()
        callback = Mock()
        tags.call_gate(15.0, callback)
        assert node.timers[0].period == 15.0
        answer(node.futures[0], success)
        callback.assert_called_once_with(success)
        assert node.destroyed == node.timers and tags.pending is None


def test_no_answer_in_time_is_a_miss_and_a_late_answer_is_ignored():
    node, tags = helper()
    callback = Mock()
    tags.call_gate(15.0, callback)
    node.timers[0].fire()
    answer(node.futures[0], True)
    callback.assert_called_once_with(False)


def test_failed_call_is_a_miss():
    node, tags = helper()
    callback = Mock()
    tags.call_gate(15.0, callback)
    answer(node.futures[0], error=RuntimeError('service died'))
    callback.assert_called_once_with(False)


def test_cancel_drops_the_pending_call():
    node, tags = helper()
    callback = Mock()
    tags.call_gate(15.0, callback)
    tags.cancel()
    answer(node.futures[0], True)
    node.timers[0].fire()
    callback.assert_not_called()
    assert node.destroyed == node.timers


def test_current_pose_is_none_without_a_transform():
    node, tags = helper()
    assert tags.current_pose() is None
