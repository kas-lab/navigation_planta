(define (problem navigation)
  (:domain navigation )

  (:objects
    wp1 wp2 wp3 wp4 wp5 wp6 wp7 wp8 wp9 wp10 - waypoint
  )

  (:init
    (robot_at wp1)

    ; (corridor wp_initial wp1)
    (corridor wp1 wp2)
    (corridor wp2 wp3)
    (corridor wp3 wp4)
    (corridor wp3 wp8)
    (corridor wp4 wp5)
    (corridor wp4 wp7)
    (corridor wp5 wp6)
    (corridor wp6 wp7)
    (corridor wp7 wp8)
    (corridor wp8 wp9)
    (corridor wp9 wp10)
    (corridor wp10 wp1)

    (dark_corridor wp1 wp2)
    (dark_corridor wp3 wp4)
    (dark_corridor wp4 wp7)
    (dark_corridor wp9 wp8)

    ; (unsafe_corridor wp1 wp2)
    ; (unsafe_corridor wp1 wp10)
    ; (unsafe_corridor wp3 wp8)
    ; (unsafe_corridor wp4 wp7)

    ; (corridor wpf wp1)
    ; (corridor wpf wp2)
    ; (corridor wpf wp3)
    ; (corridor wpf wp4)  
  )

  (:goal
    (robot_at wp4)
  )
)