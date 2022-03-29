#!/usr/bin/env python


from profi.robot.simple_move import SimpleMover
from profi.camera.camera import CameraFactory

if __name__ == "__main__":
    camera = CameraFactory.create_camera()
    simple_mover = SimpleMover(camera)

    simple_mover.spin()

