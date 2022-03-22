(define (problem robots-move-1)
(:domain robots-move)
(:objects 
  A B C D - room
  DoorA DoorB DoorC DoorD - door
  Corridor - corridor
  b1 b2 b3 b4 b5 - ball
  robby walle - robot
  gripper1 gripper2 gripper3 - gripper 
)
(:init
  (robot_at robby D)
  (robot_at walle D)
  (gripper_at gripper1 robby)
  (gripper_at gripper2 robby)
  (gripper_at gripper3 walle)
  (gripper_free gripper1)
  (gripper_free gripper2)
  (gripper_free gripper3)
  (object_at b1 D)
  (object_at b2 D)
  (object_at b3 D)
  (object_at b4 D)
  (object_at b5 D)
  (connected A Corridor DoorA )
  (connected B Corridor DoorB )
  (connected C Corridor DoorC )
  (connected D Corridor DoorD )
  (connected Corridor A DoorA )
  (connected Corridor B DoorB )
  (connected Corridor C DoorC )
  (connected Corridor D DoorD )
  (close DoorA)
  (close DoorB)
  (close DoorC)
  (close DoorD)
)

(:goal (and
    (object_at b1 A)
    (object_at b2 A)
    (object_at b3 A)
    (object_at b4 A)
    (object_at b5 A)
  )
)

)
