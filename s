(defdomain grace-fish
  (

    (:operator (!communicate ?grace ?fumin ?pool_depth ?t)
               ()
               ((recorded ?grace ?pool_depth))
               ((communicated_depth ?grace ?fumin ?pool_depth)) )

    (:operator (!raise ?grace ?pool_depth ?t)
               ()
               ((not(at_pooldepth ?grace ?pool_depth))(recorded ?grace ?pool_depth))
               ((at_pooldepth ?grace ?pool_depth)) )

    (:operator (!senseDepth ?grace ?pool_depth ?t)
               ()
               ((at_pooldepth ?grace ?pool_depth)(not(recorded ?grace ?pool_depth)))
               ((recorded ?grace ?pool_depth)) )

    (:operator (!dive ?grace ?pool_depth ?t)
               ()
               ((not(at_pooldepth ?grace ?pool_depth))(not(recorded ?grace ?pool_depth)))
               ((at_pooldepth ?grace ?pool_depth)))



    (:method (achieve-goals (list ?goal . ?goals))
     ()
     ((achieve-goal ?goal)(achieve-goals list ?goals)))

    (:method (achieve-goals nil ?goal)
             ()
             ((achieve-goal ?goal)))

   (:method (achieve-goals list nil)
            ()
            ())


   (:method (achieve-goal (communicated_depth ?grace ?fumin ?pool_depth))

             ;; If the
             ((at_pooldepth ?grace ?surface) (recorded ?grace ?pool_depth)
             )
             ((!communicate ?grace ?fumin ?pool_depth ?t))

             ((at_pooldepth ?grace ?surface) (not(recorded ?grace ?pool_depth))
              )
             ((!dive ?grace ?very_shallow ?t1)
              (!dive ?grace ?shallow ?t2)
              (!dive ?grace ?medium ?t3)
              (!dive ?grace ?deep ?t4)
              (!dive ?grace ?very_deep ?t5)
              (!senseDepth ?grace ?pool_depth ?t6)
              (!raise ?grace ?deep ?t7)
              (!raise ?grace ?medium ?t8)
              (!raise ?grace ?shallow ?t9)
              (!raise ?grace ?very_shallow ?t10)
              (!raise ?grace ?surface ?t11)
              (!communicate ?grace ?fumin ?pool_depth ?t12))

             ((atBottom ?grace) (not(knows ?grace ?pool_depth))
              (not(atSurface ?grace)))
             ((!senseDepth ?grace ?pool_depth)
              (!raise ?grace ?pool_depth)
              (!communicatePoolDepth ?grace ?fumin ?pool_depth))

             ((atBottom ?grace) (knows ?grace ?pool_depth)
              (not(atSurface ?grace)))
             ((!raise ?grace ?pool_depth)
              (!communicatePoolDepth ?grace ?fumin ?pool_depth)))

  (:- (atSurface ?x) ((not (atBottom ?x))))
  (:- (atBottom ?x) ((not (atSurface ?x)))) ))
