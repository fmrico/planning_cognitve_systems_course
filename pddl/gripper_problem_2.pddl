(define (problem strips-gripper2)
(:domain gripper-strips)
(:objects 
  rooma roomb ball1 ball2 left right
)
(:init 
  (room rooma)
  (room roomb)
  (ball ball1)
  (ball ball2)
  (ball ball3)
  (gripper left)
  (gripper right)
  (at-robby rooma)
  (free left)
  (free right)
  (at ball1 rooma)
  (at ball2 rooma)
  (at ball3 rooma)
)

(:goal 
  (and 
    (at ball1 roomb)
    (at ball2 roomb)
    (at ball3 roomb)
  )
)
)
