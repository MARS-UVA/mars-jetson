"""Command eligibility, priority selection, and monotonic watchdogs."""

import time


class Arbitration:
    """Ownership limits eligibility; priority only compares eligible sources."""

    def __init__(self, priorities, timeout=0.5, clock=time.monotonic):
        self.priorities = priorities
        self.timeout = timeout
        self.clock = clock
        self.allowed = set()
        self.locked = True
        self.stop_lock = False
        self.commands = {}

    def select(self, sources, locked):
        unknown = set(sources) - self.priorities.keys()
        if unknown:
            raise ValueError(f'Unknown command sources: {unknown}')
        # Never replay commands from before a new ownership grant.
        self.commands.clear()
        self.allowed = set(sources)
        self.locked = locked

    def receive(self, resource, source, command):
        self.commands[resource, source] = (self.clock(), command)

    def outputs(self):
        """Select one source for both outputs; missing/stale channels are zero."""
        if self.locked or self.stop_lock:
            return {}
        now = self.clock()
        fresh = {(channel, source): command
                 for (channel, source), (stamp, command) in self.commands.items()
                 if source in self.allowed and now - stamp < self.timeout}
        eligible = {source for _, source in fresh}
        if not eligible:
            return {}
        winner = max(sorted(eligible), key=lambda source: self.priorities[source])
        return {channel: command for (channel, source), command in fresh.items()
                if source == winner}

    def output(self, resource):
        return self.outputs().get(resource)
