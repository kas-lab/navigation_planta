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
    numerical-object
  )

  (:constants
    wpf - waypoint
    1.0_decimal 0.8_decimal 0.0_decimal - numerical-object
  )

  (:predicates
    (robot_at ?wp - waypoint)
    (corridor ?wp1 ?wp2 - waypoint)
    (dark_corridor ?wp1 ?wp2 - waypoint)
    (unsafe_corridor ?wp1 ?wp2 - waypoint)
    
    (inferred-corridor ?wp1 ?wp2 - waypoint)
    (inferred-dark_corridor ?wp1 ?wp2 - waypoint)
    (inferred-unsafe_corridor ?wp1 ?wp2 - waypoint)
  )
  
  (:derived (inferred-corridor ?wp1 ?wp2 - waypoint)
    (and
      (corridor ?wp1 ?wp2)
    ) 
  )
  (:derived (inferred-corridor ?wp1 ?wp2 - waypoint)
    (and
      (corridor ?wp2 ?wp1)
    ) 
  )

  (:derived (inferred-dark_corridor ?wp1 ?wp2 - waypoint)
    (and
      (dark_corridor ?wp1 ?wp2)
    ) 
  )
  (:derived (inferred-dark_corridor ?wp1 ?wp2 - waypoint)
    (and
      (dark_corridor ?wp2 ?wp1)
    ) 
  )
  
  (:derived (inferred-unsafe_corridor ?wp1 ?wp2 - waypoint)
    (and
      (unsafe_corridor ?wp1 ?wp2)
    ) 
  )
  (:derived (inferred-unsafe_corridor ?wp1 ?wp2 - waypoint)
    (and
      (unsafe_corridor ?wp2 ?wp1)
    ) 
  )

  (:action move
    :parameters (?wp1 ?wp2 - waypoint)
    :precondition (and
      (robot_at ?wp1)
      (inferred-corridor ?wp1 ?wp2)
      (not (inferred-dark_corridor ?wp1 ?wp2))
      (not (inferred-unsafe_corridor ?wp1 ?wp2))
    )
    :effect (and
      (not(robot_at ?wp1))
      (robot_at ?wp2)
    )
  )
  
  (:action move_dark
    :parameters (?wp1 ?wp2 - waypoint)
    :precondition (and
      (robot_at ?wp1)
      (inferred-corridor ?wp1 ?wp2)
      (inferred-dark_corridor ?wp1 ?wp2)
      (not (inferred-unsafe_corridor ?wp1 ?wp2))
    )
    :effect (and
      (not(robot_at ?wp1))
      (robot_at ?wp2)
    )
  )  
  
  (:action move_unsafe
    :parameters (?wp1 ?wp2 - waypoint)
    :precondition (and
      (robot_at ?wp1)
      (inferred-corridor ?wp1 ?wp2)
      (not (inferred-dark_corridor ?wp1 ?wp2))
      (inferred-unsafe_corridor ?wp1 ?wp2)
    )
    :effect (and
      (not(robot_at ?wp1))
      (robot_at ?wp2)
    )
  )
  
  (:action move_dark_unsafe
    :parameters (?wp1 ?wp2 - waypoint)
    :precondition (and
      (robot_at ?wp1)
      (inferred-corridor ?wp1 ?wp2)
      (inferred-dark_corridor ?wp1 ?wp2)
      (inferred-unsafe_corridor ?wp1 ?wp2)
    )
    :effect (and
      (not(robot_at ?wp1))
      (robot_at ?wp2)
    )
  )

)