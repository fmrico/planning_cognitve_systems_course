(define (domain briefcase-world)
(:requirements :strips :equality :typing :conditional-effects)
(:types location physob)

(:constants B - physob)

(:predicates 
  (at ?x - physob ?l - location)
  (in ?x - physob)
  (place ?l - location)
  (object ?o - physob)

)
(:action mov-b
  :parameters (?m ?l - location)
  :precondition (and (at B ?m) (not (= ?m ?l)))
  :effect (and 
    (at b ?l) (not (at B ?m))
    (forall (?z - physob)
      (when (and (in ?z) (not (= ?z B)))
        (and (at ?z ?l) (not (at ?z ?m)))))
  ) 
)

(:action put-in
  :parameters (?x - physob ?l - location)
  :precondition (not (= ?x B))
  :effect (when 
    (and (at ?x ?l) (at B ?l))
      (in ?x)
  ) 
)

(:action take-out
  :parameters (?x - physob)
  :precondition (not (= ?x B))
  :effect (not (in ?x)) 
)

)

