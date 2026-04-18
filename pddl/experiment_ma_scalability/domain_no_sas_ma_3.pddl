(define (domain navigation-domain)
  (:requirements
    :strips
    :typing
    :negative-preconditions
  )

  (:types
    waypoint
  )

  (:predicates
    (robot_at ?wp - waypoint)
    (corridor ?wp1 ?wp2 - waypoint)
    (inspection_waypoint ?wp - waypoint)
    (inspection_done)
    (delivery_waypoint ?wp - waypoint)
    (delivery_done)
  )

  (:action move
    :parameters (?wp1 ?wp2 - waypoint)
    :precondition (and
      (robot_at ?wp1)
      (corridor ?wp1 ?wp2)
    )
    :effect (and
      (not (robot_at ?wp1))
      (robot_at ?wp2)
    )
  )

  (:action a_inspect
    :parameters (?wp - waypoint)
    :precondition (and
      (robot_at ?wp)
      (inspection_waypoint ?wp)
      (not (inspection_done))
    )
    :effect (inspection_done)
  )

  (:action a_deliver
    :parameters (?wp - waypoint)
    :precondition (and
      (robot_at ?wp)
      (delivery_waypoint ?wp)
      (not (delivery_done))
    )
    :effect (delivery_done)
  )
)
