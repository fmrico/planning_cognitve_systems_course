(define (problem travel-problem)
  (:domain travel)

  ;; Instantiate the objects.
  (:objects 
    minicooper - vehicle
    alcorcon leganes fuenlabrada mostoles madrid - city
  )

  (:init
    ; Define the initial state predicates.
    (vehicle_at minicooper fuenlabrada)

    (connected alcorcon leganes)
    (connected leganes alcorcon)
    (connected mostoles alcorcon)
    (connected alcorcon mostoles)
    (connected fuenlabrada leganes)
    (connected leganes fuenlabrada)
    (connected mostoles fuenlabrada)
    (connected fuenlabrada mostoles)
    (connected madrid leganes)
    (connected leganes madrid)
    (connected madrid alcorcon)
    (connected alcorcon madrid)

    ; Define static functions
    (= (speed minicooper) 100)

    (= (distance alcorcon leganes) 10)
    (= (distance leganes alcorcon) 10)
    (= (distance mostoles alcorcon) 5)
    (= (distance alcorcon mostoles) 5)
    (= (distance fuenlabrada leganes) 100)
    (= (distance leganes fuenlabrada) 100)
    (= (distance mostoles fuenlabrada)  10)
    (= (distance fuenlabrada mostoles) 10)
    (= (distance madrid leganes) 15)
    (= (distance leganes madrid) 15)
    (= (distance madrid alcorcon) 15)
    (= (distance alcorcon madrid) 15)
  )

  (:goal (and
    (vehicle_at minicooper madrid)
  ))

 ; (:metric minimize (distance_traveled))
)
