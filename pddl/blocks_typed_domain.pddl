(define (domain blocks)
(:requirements :strips :equality :typing)

(:types
  table block - object
)
(:predicates 
  (on ?x ?y - object)
  (clear ?x - object)
)

(:constants Table - table)

(:action move
  :parameters (?b - block ?x - object ?y - block)
  :precondition 
    (and 
      (clear ?b)
      (clear ?y)
      (on ?b ?x)
      (not (= ?b ?y))
    )
  :effect 
    (and 
      (on ?b ?y)
      (clear ?x)
      (not (on ?b ?x))
      (not (clear ?y))
    )
)

(:action move_to_table
  :parameters (?b ?x - block)
  :precondition 
    (and 
      (on ?b ?x)
      (clear ?b)
      (not (= ?b ?x))
    )
  :effect 
    (and 
      (on ?b Table)
      (clear ?x)
      (not (on ?b ?x))
    )
)
)

