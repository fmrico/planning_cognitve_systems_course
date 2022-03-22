(define (problem interstellar0)
(:domain interstellar)
(:objects 
  r2d2 c3po - robot
  xwing millenumfalcon - spaceship
  naboo tatooine kessel alderaan - planet
)
(:init 
  (robot_at_planet r2d2 naboo)
  (robot_at_planet c3po tatooine)
  (spaceship_at_planet millenumfalcon alderaan)
  (spaceship_at_planet xwing kessel)
  (planet_available naboo)
  (planet_available tatooine)
  (spaceship_available xwing)
  (spaceship_available millenumfalcon)
)

(:goal (and
  (robot_at_planet r2d2 tatooine)
  (robot_at_planet c3po naboo)
  )
)

)
