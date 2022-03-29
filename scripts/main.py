#!/usr/bin/env python
from profi.robot.simple_move import SimpleMoverFactory


if __name__ == "__main__":
    simple_mover = SimpleMoverFactory.create()
    simple_mover.spin()

