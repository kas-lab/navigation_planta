(define (domain navigation-domain)
  (:requirements
    :strips
    :typing
    :adl
    :negative-preconditions
  )

  (:types
    waypoint
    numerical-object
  )

  (:predicates
    (robot_at ?wp - waypoint)
    (corridor ?wp1 ?wp2 - waypoint)
    (dark_corridor ?wp1 ?wp2 - waypoint)
    (unsafe_corridor ?wp1 ?wp2 - waypoint)
    
    (inferred-corridor ?wp1 ?wp2 - waypoint)
    (inferred-dark_corridor ?wp1 ?wp2 - waypoint)
    (inferred-unsafe_corridor ?wp1 ?wp2 - waypoint)

    (safety_requirement ?wp1 ?wp2 ?v)
    (light_requirement ?wp1 ?wp2 ?v)

    (function_nfr_satisfied ?f - inferred-Function ?qa - inferred-QualityAttributeType ?v - numerical-object)
    (fd_nfr_satisfied ?fd - inferred-FunctionDesign ?qa - inferred-QualityAttributeType ?v - numerical-object)
  )

  (:derived (function_nfr_satisfied ?f ?qa ?v)
		(exists (?fd ?qae ?qav) 
			(and 
				(inferred-SolvesF ?fd ?f)
				(not (= ?fd fd_unground))
				(inferred-FunctionGrounding ?f ?fd)
				(inferred-HasQAestimation ?fd ?qae)
				(inferred-IsQAtype ?qae ?qa)
				(inferred-Qa_has_value ?qae ?qav)
				(or
				  (lessThan ?v ?qav)
				  (equalTo ?v ?qav)
				)				
			)
		)
  )
  
  (:derived (fd_nfr_satisfied ?fd ?qa ?v)
		(exists (?qae ?qav) 
			(and 
				(not (= ?fd fd_unground))
				(inferred-HasQAestimation ?fd ?qae)
				(inferred-IsQAtype ?qae ?qa)
				(inferred-Qa_has_value ?qae ?qav)
				(or
				  (lessThan ?v ?qav)
				  (equalTo ?v ?qav)
				)				
			)
		)
  )
  
  (:derived (safety_requirement ?wp1 ?wp2 - waypoint ?v - numerical-object)
    (and
      (inferred-unsafe_corridor ?wp1 ?wp2)
      (= ?v 0.8_decimal)
    ) 
  )
  
  (:derived (safety_requirement ?wp1 ?wp2 - waypoint ?v - numerical-object)
    (and
      (not (inferred-unsafe_corridor ?wp1 ?wp2))
      (= ?v 0.0_decimal)
    ) 
  )

  (:derived (light_requirement ?wp1 ?wp2 - waypoint ?v - numerical-object)
    (and
      (inferred-dark_corridor ?wp1 ?wp2)
      (= ?v 1.0_decimal)
    ) 
  )
  
  (:derived (light_requirement ?wp1 ?wp2 - waypoint ?v - numerical-object)
    (and
      (not (inferred-dark_corridor ?wp1 ?wp2))
      (= ?v 0.0_decimal)
    ) 
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

  (:action reconfigure
    :parameters (?f ?fd_initial ?fd_goal)
    :precondition (and
      (not (= ?fd_initial ?fd_goal))

      (Function ?f)
      (FunctionDesign ?fd_initial)
      (functionGrounding ?f ?fd_initial)

      (inferred-SolvesF ?fd_goal ?f)
      (FunctionDesign ?fd_goal)
      (not (inferred-Fd_realisability ?fd_goal false_boolean))

    )
    :effect (and
      (not (functionGrounding ?f ?fd_initial))
      (functionGrounding ?f ?fd_goal)
    )
  )

  (:action reconfigure
    :parameters (?f ?fd_goal)
    :precondition (and
      (Function ?f)
      (inferred-SolvesF ?fd_goal ?f)
      (FunctionDesign ?fd_goal)
      (not (inferred-Fd_realisability ?fd_goal false_boolean))
      (not
        (exists (?fd)
          (and
            (inferred-SolvesF ?fd ?f)
            (FunctionDesign ?fd)
            (functionGrounding ?f ?fd)
          )
        )
      )
    )
    :effect (and
      (functionGrounding ?f ?fd_goal)
    )
  )

  (:action move
    :parameters (?wp1 ?wp2 - waypoint ?safety_requirement ?light_requirement - numerical-object)
    :precondition (and
      (robot_at ?wp1)
      (inferred-corridor ?wp1 ?wp2)
      (safety_requirement ?wp1 ?wp2 ?safety_requirement)
      (light_requirement ?wp1 ?wp2 ?light_requirement)
      (exists (?a ?f1 ?fd1)
        (and
          (inferred-Action ?a)
          (= ?a a_move)
          (inferred-requiresF ?a ?f1)
          (inferred-F_active ?f1 true_boolean)
          (inferred-FunctionGrounding ?f1 ?fd1)
          (fd_nfr_satisfied ?fd1 qa_accuracy ?safety_requirement)	
          (fd_nfr_satisfied ?fd1 qa_environment_light ?light_requirement)
          (not 
            (exists (?fd2)
              (and
                (not (= ?fd2 ?fd1))
                (inferred-SolvesF ?fd2 ?f1)
                (not (inferred-Fd_realisability ?fd2 false_boolean))
                (inferred-FdBetterUtility  ?fd2 ?fd1)
                (fd_nfr_satisfied ?fd2 qa_accuracy ?safety_requirement)	
                (fd_nfr_satisfied ?fd2 qa_environment_light ?light_requirement)
              )
            )
          )
        )
      )
    )
    :effect (and
      (not(robot_at ?wp1))
      (robot_at ?wp2)
    )
  )

)