(define (problem get-paid)
  (:domain briefcase-world)

(:objects 
  home office - location
  p d - physob
)
(:init 
  (place home) 
  (place office)
  (object p) (object d) (object b)
  (at B home) (at P home) (at D home) 
  (in P)
)


(:goal (and 
  (at B office) 
  (at D office) 
  (at P home))
)
)