(define (problem blocks1)
(:domain blocks)
(:objects 
  a b c d - block
)
(:init 
  (on a Table)
  (on c Table)
  (on b a)
  (on d c)
  (clear d)
  (clear b)

)

(:goal (and
  (on a Table)
  (on b a)
  (on c b)
  (on d c)
  )
)

)
