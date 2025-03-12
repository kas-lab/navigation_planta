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
		action
		component
		function
		function-design
		qa-value
		quality-attribute-type
	)

	(:constants
		fd_amcl_kinect fd_amcl_lidar fd_aruco fd_aruco_headlamp fd_mrpt_kinect fd_mrpt_lidar fd_unground - function-design 
		qa_performance_zero qa_v_accuracy_bad qa_v_accuracy_excellent qa_v_accuracy_good qa_v_accuracy_medium qa_v_accuracy_really_good qa_v_energy_cost_bad qa_v_energy_cost_excellent qa_v_energy_cost_good qa_v_energy_cost_medium qa_v_energy_cost_really_bad qa_v_energy_cost_really_good qa_v_environment_light_bright qa_v_environment_light_low - qa-value 
		obs_environment_light performance qa_accuracy qa_energy_cost qa_environment_light - quality-attribute-type 
		c_camera c_headlamp c_kinect c_lidar - component 
		a_move - action 
		f_localization - function 
		0.9_decimal - numerical-object 
		ERROR_string false_boolean true_boolean - object
	)

  (:predicates
    (robot_at ?wp - waypoint)
    (corridor ?wp1 ?wp2 - waypoint)
    (dark_corridor ?wp1 ?wp2 - waypoint)
    (unsafe_corridor ?wp1 ?wp2 - waypoint)
    
    (safety_requirement ?wp1 ?wp2 ?v)
    (light_requirement ?wp1 ?wp2 ?v)

    (fd_nfr_satisfied ?fd - inferred-FunctionDesign ?qa - inferred-QualityAttributeType ?v - numerical-object)

		(Action ?x)
		(Component ?x)
		(Function ?x)
		(FunctionDesign ?x)
		(QAvalue ?x)
		(QualityAttributeType ?x)
		(c_status ?x ?y)
		(differentFrom ?x ?y)
		(f_active ?x ?y)
		(fdBetterUtility ?x ?y)
		(fd_realisability ?x ?y)
		(fd_utility ?x ?y)
		(functionGrounding ?x ?y)
		(hasQAestimation ?x ?y)
		(inferred-Action ?x)
		(inferred-C_status ?x ?y)
		(inferred-Component ?x)
		(inferred-DifferentFrom ?x ?y)
		(inferred-F_active ?x ?y)
		(inferred-FdBetterUtility ?x ?y)
		(inferred-Fd_realisability ?x ?y)
		(inferred-Fd_utility ?x ?y)
		(inferred-Function ?x)
		(inferred-FunctionDesign ?x)
		(inferred-FunctionGrounding ?x ?y)
		(inferred-HasQAestimation ?x ?y)
		(inferred-Inconsistent)
		(inferred-IsQAtype ?x ?y)
		(inferred-QAvalue ?x)
		(inferred-Qa_has_value ?x ?y)
		(inferred-QualityAttributeType ?x)
		(inferred-RequiresC ?x ?y)
		(inferred-RequiresF ?x ?y)
		(inferred-SameAs ?x ?y)
		(inferred-SolvesF ?x ?y)
		(isQAtype ?x ?y)
		(lessThan ?x ?y)
		(qa_has_value ?x ?y)
		(requiresC ?x ?y)
		(requiresF ?x ?y)
		(sameAs ?x ?y)
		(solvesF ?x ?y)
		(equalTo ?x ?y)
  )


(:derived (inferred-Action ?x - action) 
	(and
		(Action ?x)
	)
)


(:derived (inferred-Action ?x - action) 
	(exists (?y - function)
 		(and
			(inferred-RequiresF ?x ?y)
		)
	)
 )


(:derived (inferred-C_status ?x - component ?y) 
	(and
		(c_status ?x ?y)
	)
)


(:derived (inferred-Component ?x - component) 
	(and
		(Component ?x)
	)
)


(:derived (inferred-Component ?x - component) 
	(exists (?y)
 		(and
			(inferred-C_status ?x ?y)
		)
	)
 )


(:derived (inferred-Component ?y - component) 
	(exists (?x - function-design)
 		(and
			(inferred-RequiresC ?x ?y)
		)
	)
 )


(:derived (inferred-DifferentFrom ?x ?y) 
	(and
		(differentFrom ?x ?y)
	)
)


(:derived (inferred-F_active ?f - function ?true_boolean) 
	(and
		(= ?true_boolean true_boolean)
		(exists (?fd - function-design)
 			(and
				(inferred-Function ?f)
				(inferred-FunctionDesign ?fd)
				(not (= ?fd fd_unground))
				(inferred-SolvesF ?fd ?f)
				(inferred-FunctionGrounding ?f ?fd)
			)
		)
 	)
 )


(:derived (inferred-F_active ?x - function ?y) 
	(and
		(f_active ?x ?y)
	)
)


(:derived (inferred-FdBetterUtility ?fd_better - function-design ?fd - function-design) 
	(exists (?x_better - numerical-object ?f - function ?x - numerical-object)
 		(and
			(inferred-Fd_utility ?fd_better ?x_better)
			(inferred-Function ?f)
			(inferred-FunctionDesign ?fd)
			(inferred-FunctionDesign ?fd_better)
			(lessThan ?x ?x_better)
			(inferred-SolvesF ?fd ?f)
			(inferred-Fd_utility ?fd ?x)
			(inferred-SolvesF ?fd_better ?f)
		)
	)
 )


(:derived (inferred-FdBetterUtility ?x - function-design ?y - function-design) 
	(and
		(fdBetterUtility ?x ?y)
	)
)


(:derived (inferred-Fd_realisability ?fd - function-design ?false_boolean) 
	(and
		(= ?false_boolean false_boolean)
		(exists (?c - component)
 			(and
				(inferred-RequiresC ?fd ?c)
				(inferred-Component ?c)
				(inferred-C_status ?c ERROR_string)
			)
		)
 	)
 )


(:derived (inferred-Fd_realisability ?fd1 - function-design ?false_boolean) 
	(and
		(= ?false_boolean false_boolean)
		(exists (?mqa - qa-value ?mqav - qa-value)
 			(and
				(inferred-FunctionDesign ?fd1)
				(= ?fd1 fd_aruco)
				(inferred-QAvalue ?mqa)
				(= ?mqa obs_environment_light)
				(inferred-Qa_has_value ?mqa ?mqav)
				(inferred-IsQAtype ?mqa qa_environment_light)
				(lessThan ?mqav 0.9_decimal)
			)
		)
 	)
 )


(:derived (inferred-Fd_realisability ?x - function-design ?y) 
	(and
		(fd_realisability ?x ?y)
	)
)


(:derived (inferred-Fd_utility ?fd - function-design ?qav - qa-value) 
	(exists (?f - function ?qa - quality-attribute-type)
 		(and
			(inferred-Function f_localization)
			(inferred-FunctionDesign ?fd)
			(inferred-SolvesF ?fd ?f)
			(inferred-HasQAestimation ?fd ?qa)
			(inferred-IsQAtype ?qa qa_accuracy)
			(inferred-Qa_has_value ?qa ?qav)
		)
	)
 )


(:derived (inferred-Fd_utility ?x - function-design ?y - qa-value) 
	(and
		(fd_utility ?x ?y)
	)
)


(:derived (inferred-Function ?x - function) 
	(and
		(Function ?x)
	)
)


(:derived (inferred-Function ?x - function) 
	(exists (?y)
 		(and
			(inferred-F_active ?x ?y)
		)
	)
 )


(:derived (inferred-Function ?x - function) 
	(exists (?y - function-design)
 		(and
			(inferred-FunctionGrounding ?x ?y)
		)
	)
 )


(:derived (inferred-Function ?y - function) 
	(exists (?x - action)
 		(and
			(inferred-RequiresF ?x ?y)
		)
	)
 )


(:derived (inferred-Function ?y - function) 
	(exists (?x - function-design)
 		(and
			(inferred-SolvesF ?x ?y)
		)
	)
 )


(:derived (inferred-FunctionDesign ?x - function-design) 
	(and
		(FunctionDesign ?x)
	)
)


(:derived (inferred-FunctionDesign ?x - function-design) 
	(exists (?y - function-design)
 		(and
			(inferred-FdBetterUtility ?x ?y)
		)
	)
 )


(:derived (inferred-FunctionDesign ?x - function-design) 
	(exists (?y)
 		(and
			(inferred-Fd_realisability ?x ?y)
		)
	)
 )


(:derived (inferred-FunctionDesign ?x - function-design) 
	(exists (?y)
 		(and
			(inferred-Fd_utility ?x ?y)
		)
	)
 )


(:derived (inferred-FunctionDesign ?x - function-design) 
	(exists (?y - qa-value)
 		(and
			(inferred-HasQAestimation ?x ?y)
		)
	)
 )


(:derived (inferred-FunctionDesign ?x - function-design) 
	(exists (?y - component)
 		(and
			(inferred-RequiresC ?x ?y)
		)
	)
 )


(:derived (inferred-FunctionDesign ?x - function-design) 
	(exists (?y - function)
 		(and
			(inferred-SolvesF ?x ?y)
		)
	)
 )


(:derived (inferred-FunctionDesign ?y - function-design) 
	(exists (?x)
 		(and
			(inferred-FdBetterUtility ?x ?y)
		)
	)
 )


(:derived (inferred-FunctionDesign ?y - function-design) 
	(exists (?x - function)
 		(and
			(inferred-FunctionGrounding ?x ?y)
		)
	)
 )


(:derived (inferred-FunctionGrounding ?x - function ?y - function-design) 
	(and
		(functionGrounding ?x ?y)
	)
)


(:derived (inferred-HasQAestimation ?x - function-design ?y - qa-value) 
	(and
		(hasQAestimation ?x ?y)
	)
)


(:derived (inferred-Inconsistent ) 
	(exists (?x ?y ?z)
 		(and
			(inferred-C_status ?x ?y)
			(inferred-C_status ?x ?z)
			(not (= ?y ?z))
		)
	)
 )


(:derived (inferred-Inconsistent ) 
	(exists (?x ?y ?z)
 		(and
			(inferred-F_active ?x ?y)
			(inferred-F_active ?x ?z)
			(not (= ?y ?z))
		)
	)
 )


(:derived (inferred-Inconsistent ) 
	(exists (?x ?y ?z)
 		(and
			(inferred-Fd_realisability ?x ?y)
			(inferred-Fd_realisability ?x ?z)
			(not (= ?y ?z))
		)
	)
 )


(:derived (inferred-Inconsistent ) 
	(exists (?x ?y ?z)
 		(and
			(inferred-Fd_utility ?x ?y)
			(inferred-Fd_utility ?x ?z)
			(not (= ?y ?z))
		)
	)
 )


(:derived (inferred-Inconsistent ) 
	(exists (?x ?y ?z)
 		(and
			(inferred-IsQAtype ?x ?y)
			(inferred-IsQAtype ?x ?z)
			(= ?y ?z)
		)
	)
 )


(:derived (inferred-Inconsistent ) 
	(exists (?x ?y ?z)
 		(and
			(inferred-Qa_has_value ?x ?y)
			(inferred-Qa_has_value ?x ?z)
			(not (= ?y ?z))
		)
	)
 )


(:derived (inferred-Inconsistent ) 
	(exists (?x ?y ?z)
 		(and
			(inferred-SolvesF ?x ?y)
			(inferred-SolvesF ?x ?z)
			(= ?y ?z)
		)
	)
 )


(:derived (inferred-IsQAtype ?x - qa-value ?y - quality-attribute-type) 
	(and
		(isQAtype ?x ?y)
	)
)


(:derived (inferred-QAvalue ?x - qa-value) 
	(and
		(QAvalue ?x)
	)
)


(:derived (inferred-QAvalue ?x - qa-value) 
	(exists (?y - quality-attribute-type)
 		(and
			(inferred-IsQAtype ?x ?y)
		)
	)
 )


(:derived (inferred-QAvalue ?x - qa-value) 
	(exists (?y - numerical-object)
 		(and
			(inferred-Qa_has_value ?x ?y)
		)
	)
 )


(:derived (inferred-QAvalue ?y - qa-value) 
	(exists (?x - function-design)
 		(and
			(inferred-HasQAestimation ?x ?y)
		)
	)
 )


(:derived (inferred-Qa_has_value ?x - qa-value ?y - numerical-object) 
	(and
		(qa_has_value ?x ?y)
	)
)


(:derived (inferred-QualityAttributeType ?x - quality-attribute-type) 
	(and
		(QualityAttributeType ?x)
	)
)


(:derived (inferred-QualityAttributeType ?y - quality-attribute-type) 
	(exists (?x - qa-value)
 		(and
			(inferred-IsQAtype ?x ?y)
		)
	)
 )


(:derived (inferred-RequiresC ?x - function-design ?y - component) 
	(and
		(requiresC ?x ?y)
	)
)


(:derived (inferred-RequiresF ?x - action ?y - function) 
	(and
		(requiresF ?x ?y)
	)
)


(:derived (inferred-SameAs ?x - numerical-object ?y - numerical-object) 
	(and
		(sameAs ?x ?y)
	)
)


(:derived (inferred-SolvesF ?fd_unground - function-design ?f - function) 
	(and
		(= ?fd_unground fd_unground)
		(and
			(inferred-Function ?f)
			(inferred-FunctionDesign fd_unground)
		)
	)
 )


(:derived (inferred-SolvesF ?x - function-design ?y - function) 
	(and
		(solvesF ?x ?y)
	)
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
  
  (:derived (safety_requirement ?wp1 ?wp2 - waypoint ?v - numerical-object)
    (and
      (unsafe_corridor ?wp1 ?wp2)
      (= ?v 0.8_decimal)
    ) 
  )
  
  (:derived (safety_requirement ?wp1 ?wp2 - waypoint ?v - numerical-object)
    (and
      (not (unsafe_corridor ?wp1 ?wp2))
      (= ?v 0.0_decimal)
    ) 
  )

  (:derived (light_requirement ?wp1 ?wp2 - waypoint ?v - numerical-object)
    (and
      (dark_corridor ?wp1 ?wp2)
      (= ?v 1.0_decimal)
    ) 
  )
  
  (:derived (light_requirement ?wp1 ?wp2 - waypoint ?v - numerical-object)
    (and
      (not (dark_corridor ?wp1 ?wp2))
      (= ?v 0.0_decimal)
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
