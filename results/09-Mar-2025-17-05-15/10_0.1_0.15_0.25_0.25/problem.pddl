(define (problem navigation-problem)
 (:domain navigation-domain)
 (:objects
   wp0 wp1 wp2 wp3 wp4 wp5 wp6 wp7 wp8 - waypoint
 )
 (:init (corridor wp0 wp1) (corridor wp1 wp0) (corridor wp0 wp3) (corridor wp3 wp0) (dark_corridor wp0 wp3) (dark_corridor wp3 wp0) (corridor wp2 wp4) (corridor wp4 wp2) (unsafe_corridor wp2 wp4) (unsafe_corridor wp4 wp2) (corridor wp3 wp5) (corridor wp5 wp3) (unsafe_corridor wp3 wp5) (unsafe_corridor wp5 wp3) (corridor wp4 wp7) (corridor wp7 wp4) (corridor wp5 wp6) (corridor wp6 wp5) (dark_corridor wp5 wp6) (dark_corridor wp6 wp5) (corridor wp5 wp8) (corridor wp8 wp5) (corridor wp6 wp7) (corridor wp7 wp6) (robot_at wp0))
 (:goal (and (robot_at wp8)))
)
