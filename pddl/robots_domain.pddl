(define (domain robots-move)
(:requirements :strips :equality :typing)

(:types
  room corridor - location
  door
  robot
  ball
  gripper
)

(:predicates 
  (robot_at ?r - robot ?l - location)
  (object_at ?b - ball ?l - location)
  (gripper_free ?g - gripper)
  (gripper_at ?g - gripper ?r - robot)
  (robot_carry ?r - robot ?g - gripper ?o - ball)
  (connected ?l1 ?l2 - location ?d - door)
  (open ?d - door)
  (close ?d - door)
)

(:action open-door
  :parameters (?r - robot ?l1 ?l2 - location ?d - door)
  :precondition 
    (and 
      (robot_at ?r ?l1)
      (close ?d)
      (connected ?l1 ?l2 ?d)
    )
  :effect 
    (and 
      (open ?d)
      (not (close ?d))
    )
)

(:action move
  :parameters (?r - robot ?from ?to - location ?d - door)
  :precondition 
    (and 
      (robot_at ?r ?from)
      (connected ?from ?to ?d)
      (open ?d)
    )
  :effect 
    (and 
      (robot_at ?r ?to)
      (not (robot_at ?r ?from))
    )
)
(:action pick
  :parameters (?o - ball ?l - location ?r - robot ?g - gripper)
  :precondition 
    (and
      (gripper_at ?g ?r)
      (object_at ?o ?l)
      (robot_at ?r ?l) 
      (gripper_free ?g)
    )
:effect 
  (and 
    (robot_carry ?r ?g ?o) 
    (not (object_at ?o ?l))
    (not (gripper_free ?g)))
  )

(:action drop
:parameters (?o - ball ?l - location ?r - robot ?g - gripper)
:precondition 
  (and 
    (gripper_at ?g ?r)
    (robot_at ?r ?l)
    (robot_carry ?r ?g ?o)
  )
:effect 
  (and 
    (object_at ?o ?l)
    (gripper_free ?g)
    (not (robot_carry ?r ?g ?o))
  )
)
)

