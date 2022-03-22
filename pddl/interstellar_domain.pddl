(define (domain interstellar)
(:requirements :strips :equality :typing :durative-actions)

(:types
  planet spaceship robot
)
(:predicates 
  (robot_at_planet ?r - robot ?p - planet)
  (spaceship_at_planet ?s - spaceship ?p - planet)
  (robot_in_spaceship ?r - robot ?s - spaceship)
  (planet_available ?p - planet)
  (spaceship_available ?s - spaceship)
)

(:durative-action load
    :parameters (?r - robot ?s - spaceship ?p - planet)
    :duration (= ?duration 6)
    :condition (and 
        (at start (and 
          (robot_at_planet ?r ?p)
          (spaceship_available ?s)
        ))
        (over all (and 
          (spaceship_at_planet ?s ?p)
        ))
    )
    :effect (and 
        (at start (and 
          (not (robot_at_planet ?r ?p))
        ))
        (at end (and 
          (robot_in_spaceship ?r ?s)
          (not (spaceship_available ?s))
        ))
    )
)

(:durative-action unload
    :parameters (?r - robot ?s - spaceship ?p - planet)
    :duration (= ?duration 5)
    :condition (and 
        (at start (and 
          (robot_in_spaceship ?r ?s)     
        ))
        (over all (and 
          (spaceship_at_planet ?s ?p)
        ))
    )
    :effect (and 
        (at start (and 
          (not (robot_in_spaceship ?r ?s))
        ))
        (at end (and 
          (robot_at_planet ?r ?p)
          (spaceship_available ?s)
        ))
    )
)


(:durative-action fly
    :parameters (?s - spaceship ?from ?to - planet)
    :duration (= ?duration 15)
    :condition (and 
        (at start (and 
          (spaceship_at_planet ?s ?from) 
        ))
        (over all (and 
          (not (= ?from ?to))
        ))
        (at end (and
          (planet_available ?to)
        ))
    )
    :effect (and 
        (at start (and 
          (planet_available ?from)
          (not (spaceship_at_planet ?s ?from))
        ))
        (at end (and 
          (not (planet_available ?to))
          (spaceship_at_planet ?s ?to)
        ))
    )
)

)

