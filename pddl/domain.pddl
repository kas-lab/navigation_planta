(define (domain navigation)
  (:requirements
    :strips
    :typing
    :adl
    :negative-preconditions
  )

  (:types
    waypoint
    action
  )

  (:predicates
    (robot_at ?wp - waypoint)
    (connected ?wp1 ?wp2 - waypoint)
  )

  (:action move
    :parameters (?wp1 ?wp2 - waypoint)
    :precondition (and
      (robot_at ?wp1)
      (connected ?wp1 ?wp2)
    )
    :effect (and
      (not(robot_at ?wp1))
      (robot_at ?wp2)
    )
  )

)