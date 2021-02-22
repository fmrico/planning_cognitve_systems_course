(define (problem blocks1)
(:domain blocks)
(:objects 
  a b c
)
(:init 
  (block a)
  (block b)
  (block c)
  (clear b)
  (clear c)
  (on b Table)
  (on a Table)
  (on c a)
)

(:goal (and
  (on b c)(on a b))
)

)
