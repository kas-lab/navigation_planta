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
    
    (safety_requirement ?wp1 - waypoint ?wp2 - waypoint ?v - numerical-object)
    (light_requirement ?wp1 - waypoint ?wp2 - waypoint ?v - numerical-object)

    (fd_nfr_satisfied ?fd - inferred-FunctionDesign ?qa - inferred-QualityAttributeType ?v - numerical-object)
  )

  (:derived (fd_nfr_satisfied ?fd - function-design ?qa - quality-attribute-type ?v - numerical-object)
		(exists (?qae - qa-value ?qav - numerical-object) 
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
  
  (:action reconfigure
    :parameters (?f - function ?fd_initial ?fd_goal - function-design)
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
    :parameters (?f - function ?fd_goal - function-design)
    :precondition (and
      (Function ?f)
      (inferred-SolvesF ?fd_goal ?f)
      (FunctionDesign ?fd_goal)
      (not (inferred-Fd_realisability ?fd_goal false_boolean))
      (not
        (exists (?fd - function-design)
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
      (corridor ?wp1 ?wp2)
      (safety_requirement ?wp1 ?wp2 ?safety_requirement)
      (light_requirement ?wp1 ?wp2 ?light_requirement)
      (exists (?a - action ?f1 - function ?fd1 - function-design)
        (and
          (inferred-Action ?a)
          (= ?a a_move)
          (inferred-requiresF ?a ?f1)
          (inferred-F_active ?f1 true_boolean)
          (inferred-FunctionGrounding ?f1 ?fd1)
          (fd_nfr_satisfied ?fd1 qa_accuracy ?safety_requirement)	
          (fd_nfr_satisfied ?fd1 qa_environment_light ?light_requirement)
          (not 
            (exists (?fd2 - function-design)
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