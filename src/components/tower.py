import logging
from enum import Enum
import rev
from utils import units, lazytalonsrx
import numpy as np
import ctre


class Tower:

    LOW_TOWER_SPEED = 0.5
    HIGH_TOWER_SPEED = 0.5

    low_tower_motor: lazytalonsrx.LazyTalonSRX
    high_tower_motor: lazytalonsrx.LazyTalonSRX

    def __init__(self):
        self.is_indexing = False
        self.ball_count = [False, False, False, False]

    def on_enable(self):
        pass


    def startIndexing(self):
        self.is_indexing = True

    def stopIndexing(self):
        self.is_indexing = False

    def hasBall(self, index: int):
        return self.ball_count[index]

    def execute(self):
        # TODO handle tower
        # TODO track_balls
        if False:  # trigger 0
            self.ball_count[0] = True
        if False:  # trigger 1
            self.ball_count[1] = True
        if False:  # trigger 2
            self.ball_count[2] = True
        if False:  # trigger 3
            self.ball_count[3] = True
