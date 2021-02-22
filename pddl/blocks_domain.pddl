(define (domain blocks)
(:requirements :strips :equality)
(:predicates 
  (on ?x ?y)
  (clear ?x)
  (table ?t)
  (block ?b)
)

(:constants Table)

(:action move
  :parameters (?b ?x ?y)
  :precondition 
    (and 
      (block ?b) 
      (block ?y) 
      (clear ?b)
      (clear ?y)
      (on ?b ?x)
      (not (= ?b ?x))
      (not (= ?b ?y))
      (not (= ?x ?y))
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
  :parameters (?b ?x)
  :precondition 
    (and 
      (block ?b)
      (block ?x)
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

