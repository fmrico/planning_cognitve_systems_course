(define (domain moving)

  (:requirements :typing :fluents :durative-actions :universal-preconditions :conditional-effects)

  (:types good house person - object)

  (:predicates
    (good_at ?g - good ?h - house)
    (person_own ?p - person ?g - good)
    (person_at ?v - person ?h - house)
    (person_carry ?p - person ?g - good)
  )

  (:durative-action move
    :parameters (?p - person ?h1 ?h2 - house)
    :duration (= ?duration 1)
    :condition (and 
      (at start(person_at ?p ?h1))
      ;;(at start
      ;;  (forall (?g - good)
      ;;    (and(person_own ?p ?g)
      ;;    (person_carry ?p ?g))
      ;;  )
      ;;)
    )
    :effect (and 
          (at end(not (person_at ?p ?h1)))
          (at end(person_at ?p ?h2))
    )
  )

  (:durative-action pick
    :parameters (?p - person ?h - house ?g - good)
    :duration (= ?duration 1)
    :condition (and 
      (at start(person_at ?p ?h))
      (at start(good_at ?g ?h))
    )
    :effect (and 
          (at end(not (good_at ?g ?h)))
          (at end(person_carry ?p ?g))
    )
  )

  (:durative-action place
    :parameters (?p - person ?h - house ?g - good)
    :duration (= ?duration 1)
    :condition (and 
      (at start(person_at ?p ?h))
      (at start(person_carry ?p ?g))
      
    )
    :effect (and 
          (at end(good_at ?g ?h))
          (at end(not(person_carry ?p ?g)))
    )
  )

)