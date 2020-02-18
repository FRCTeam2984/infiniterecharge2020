from components import intake, tower


class Indexer:

    # required components
    intake: intake.Intake
    tower: tower.Tower

    def __init__(self):
        self.is_indexing = False

    def on_enable(self):
        pass

    def on_disable(self):
        self.stop()

    def index(self) -> None:
        """Start indexing balls."""
        self.is_indexing = True

    def stop(self) -> None:
        """Stop indexing balls."""
        self.is_indexing = False

    def execute(self):
        if self.is_indexing:
            self.intake.intake()
            self.tower.lift()
        else:
            self.intake.stop()
            self.tower.stop()
