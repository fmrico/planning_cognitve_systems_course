(define (problem moving-problem)
  (:domain moving)

  ;; Instantiate the objects.
  (:objects 
    paco - person
    house_alcorcon house_fuenlabrada - house
    tv ball - good
  )

  (:init
    (person_at paco house_fuenlabrada)

    (person_own paco tv)
    (person_own paco ball)

    (good_at tv house_fuenlabrada)
    (good_at ball house_fuenlabrada)

  )

  (:goal (and
    ;;(good_at tv house_alcorcon)
    ;;(good_at ball house_alcorcon)
    ;;(person_at paco house_alcorcon)
    (forall (?g - good) 
      (good_at ?g house_alcorcon)
    )
  ))

)
