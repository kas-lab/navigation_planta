(define (problem navigation-problem)
 (:domain navigation-domain)
 (:objects
   wp0 wp1 wp2 wp3 wp4 wp5 wp6 wp7 wp8 wp9 wp10 wp11 wp12 wp13 - waypoint
 )
 (:init (corridor wp0 wp1) (corridor wp1 wp0) (unsafe_corridor wp0 wp1) (unsafe_corridor wp1 wp0) (corridor wp0 wp4) (corridor wp4 wp0) (unsafe_corridor wp0 wp4) (unsafe_corridor wp4 wp0) (corridor wp1 wp2) (corridor wp2 wp1) (corridor wp1 wp5) (corridor wp5 wp1) (dark_corridor wp1 wp5) (dark_corridor wp5 wp1) (unsafe_corridor wp1 wp5) (unsafe_corridor wp5 wp1) (corridor wp2 wp3) (corridor wp3 wp2) (dark_corridor wp2 wp3) (dark_corridor wp3 wp2) (corridor wp2 wp6) (corridor wp6 wp2) (dark_corridor wp2 wp6) (dark_corridor wp6 wp2) (unsafe_corridor wp2 wp6) (unsafe_corridor wp6 wp2) (corridor wp4 wp8) (corridor wp8 wp4) (corridor wp5 wp6) (corridor wp6 wp5) (corridor wp5 wp9) (corridor wp9 wp5) (dark_corridor wp5 wp9) (dark_corridor wp9 wp5) (corridor wp6 wp7) (corridor wp7 wp6) (corridor wp7 wp10) (corridor wp10 wp7) (corridor wp8 wp9) (corridor wp9 wp8) (corridor wp8 wp11) (corridor wp11 wp8) (corridor wp9 wp12) (corridor wp12 wp9) (corridor wp11 wp12) (corridor wp12 wp11) (corridor wp12 wp13) (corridor wp13 wp12) (robot_at wp0))
 (:goal (and (robot_at wp13)))
)
