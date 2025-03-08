(define (problem navigation-problem)
 (:domain navigation-domain)
 (:objects
   wp0 wp1 wp2 wp3 wp4 wp5 wp6 wp7 wp8 wp9 wp10 wp11 wp12 wp13 wp14 wp15 wp16 wp17 wp18 wp19 wp20 wp21 wp22 wp23 wp24 wp25 wp26 - waypoint
 )
 (:init (corridor wp0 wp1) (dark_corridor wp0 wp1) (corridor wp0 wp4) (dark_corridor wp0 wp4) (unsafe_corridor wp0 wp4) (corridor wp2 wp3) (dark_corridor wp2 wp3) (unsafe_corridor wp2 wp3) (corridor wp2 wp6) (unsafe_corridor wp2 wp6) (corridor wp4 wp8) (corridor wp5 wp6) (dark_corridor wp5 wp6) (corridor wp6 wp7) (corridor wp6 wp10) (dark_corridor wp6 wp10) (corridor wp7 wp11) (corridor wp8 wp9) (unsafe_corridor wp8 wp9) (corridor wp8 wp12) (unsafe_corridor wp8 wp12) (corridor wp9 wp13) (corridor wp10 wp11) (corridor wp10 wp15) (unsafe_corridor wp10 wp15) (corridor wp11 wp16) (corridor wp12 wp17) (corridor wp13 wp14) (corridor wp13 wp18) (corridor wp14 wp15) (corridor wp14 wp19) (dark_corridor wp14 wp19) (corridor wp15 wp16) (dark_corridor wp15 wp16) (corridor wp15 wp20) (corridor wp16 wp21) (corridor wp17 wp18) (unsafe_corridor wp17 wp18) (corridor wp17 wp22) (corridor wp18 wp19) (unsafe_corridor wp18 wp19) (corridor wp18 wp23) (corridor wp19 wp20) (corridor wp19 wp24) (corridor wp20 wp21) (dark_corridor wp20 wp21) (corridor wp20 wp25) (corridor wp22 wp23) (corridor wp23 wp24) (corridor wp25 wp26) (robot_at wp0))
 (:goal (and (robot_at wp26)))
)
