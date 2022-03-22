(define (domain travel)

  (:requirements :typing :fluents :durative-actions)

  (:types vehicle city - object)

  (:predicates
    (vehicle_at ?v - vehicle ?c - city)
    (connected ?c1 ?c2 - city)
  )

  (:functions
    (speed ?c - vehicle)
    (distance_traveled ?c - vehicle)
    (distance ?c1 ?c2 - city)
  )

  (:durative-action drive
    :parameters (?v - vehicle ?c1 ?c2 - city)
    :duration (= ?duration 1)
    :condition (and (at start (connected ?c1 ?c2))
                    (at start (vehicle_at ?v ?c1))
                    )
    :effect (and (at start (not (vehicle_at ?v ?c1)))
                 (at end (vehicle_at ?v ?c2))
                 (at end(increase (distance_traveled ?v) (distance ?c1 ?c2)))
                 )
  )  

)