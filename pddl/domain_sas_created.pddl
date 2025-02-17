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
		1.0_decimal ERROR_string a_move c_camera c_headlamp c_kinect c_lidar f_localization false_boolean fd_amcl_kinect fd_amcl_lidar fd_aruco fd_aruco_headlamp fd_mrpt_kinect fd_mrpt_lidar fd_unground obs_environment_light performance qa_accuracy qa_energy_cost qa_environment_light qa_performance_zero qa_v_accuracy_bad qa_v_accuracy_excellent qa_v_accuracy_good qa_v_accuracy_medium qa_v_accuracy_really_good qa_v_energy_cost_bad qa_v_energy_cost_excellent qa_v_energy_cost_good qa_v_energy_cost_medium qa_v_energy_cost_really_bad qa_v_energy_cost_really_good qa_v_environment_light_bright true_boolean - object
	)

  (:predicates
    (robot_at ?wp - waypoint)
    (corridor ?wp1 ?wp2 - waypoint)

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


(:derived (inferred-Action ?x) 
	(and
		(Action ?x)
	)
)


(:derived (inferred-Action ?x) 
	(exists (?y)
 		(and
			(inferred-RequiresF ?x ?y)
		)
	)
 )


(:derived (inferred-C_status ?x ?y) 
	(and
		(c_status ?x ?y)
	)
)


(:derived (inferred-Component ?x) 
	(and
		(Component ?x)
	)
)


(:derived (inferred-Component ?x) 
	(exists (?y)
 		(and
			(inferred-C_status ?x ?y)
		)
	)
 )


(:derived (inferred-Component ?y) 
	(exists (?x)
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


(:derived (inferred-F_active ?f ?true_boolean) 
	(and
		(= ?true_boolean true_boolean)
		(exists (?fd)
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


(:derived (inferred-F_active ?x ?y) 
	(and
		(f_active ?x ?y)
	)
)


(:derived (inferred-FdBetterUtility ?fd_better ?fd) 
	(exists (?x_better ?f ?x)
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


(:derived (inferred-FdBetterUtility ?x ?y) 
	(and
		(fdBetterUtility ?x ?y)
	)
)


(:derived (inferred-Fd_realisability ?fd ?false_boolean) 
	(and
		(= ?false_boolean false_boolean)
		(exists (?c)
 			(and
				(inferred-RequiresC ?fd ?c)
				(inferred-Component ?c)
				(inferred-C_status ?c ERROR_string)
			)
		)
 	)
 )


(:derived (inferred-Fd_realisability ?fd1 ?false_boolean) 
	(and
		(= ?false_boolean false_boolean)
		(exists (?mqa ?mqav)
 			(and
				(inferred-FunctionDesign ?fd1)
				(= ?fd1 fd_aruco)
				(inferred-QAvalue ?mqa)
				(= ?mqa obs_environment_light)
				(inferred-Qa_has_value ?mqa ?mqav)
				(inferred-IsQAtype ?mqa qa_environment_light)
				(lessThan ?mqav 1.0_decimal)
			)
		)
 	)
 )


(:derived (inferred-Fd_realisability ?x ?y) 
	(and
		(fd_realisability ?x ?y)
	)
)


(:derived (inferred-Fd_utility ?fd ?qav) 
	(exists (?f ?qa)
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


(:derived (inferred-Fd_utility ?x ?y) 
	(and
		(fd_utility ?x ?y)
	)
)


(:derived (inferred-Function ?x) 
	(and
		(Function ?x)
	)
)


(:derived (inferred-Function ?x) 
	(exists (?y)
 		(and
			(inferred-F_active ?x ?y)
		)
	)
 )


(:derived (inferred-Function ?x) 
	(exists (?y)
 		(and
			(inferred-FunctionGrounding ?x ?y)
		)
	)
 )


(:derived (inferred-Function ?y) 
	(exists (?x)
 		(and
			(inferred-RequiresF ?x ?y)
		)
	)
 )


(:derived (inferred-Function ?y) 
	(exists (?x)
 		(and
			(inferred-SolvesF ?x ?y)
		)
	)
 )


(:derived (inferred-FunctionDesign ?x) 
	(and
		(FunctionDesign ?x)
	)
)


(:derived (inferred-FunctionDesign ?x) 
	(exists (?y)
 		(and
			(inferred-FdBetterUtility ?x ?y)
		)
	)
 )


(:derived (inferred-FunctionDesign ?x) 
	(exists (?y)
 		(and
			(inferred-Fd_realisability ?x ?y)
		)
	)
 )


(:derived (inferred-FunctionDesign ?x) 
	(exists (?y)
 		(and
			(inferred-Fd_utility ?x ?y)
		)
	)
 )


(:derived (inferred-FunctionDesign ?x) 
	(exists (?y)
 		(and
			(inferred-HasQAestimation ?x ?y)
		)
	)
 )


(:derived (inferred-FunctionDesign ?x) 
	(exists (?y)
 		(and
			(inferred-RequiresC ?x ?y)
		)
	)
 )


(:derived (inferred-FunctionDesign ?x) 
	(exists (?y)
 		(and
			(inferred-SolvesF ?x ?y)
		)
	)
 )


(:derived (inferred-FunctionDesign ?y) 
	(exists (?x)
 		(and
			(inferred-FdBetterUtility ?x ?y)
		)
	)
 )


(:derived (inferred-FunctionDesign ?y) 
	(exists (?x)
 		(and
			(inferred-FunctionGrounding ?x ?y)
		)
	)
 )


(:derived (inferred-FunctionGrounding ?x ?y) 
	(and
		(functionGrounding ?x ?y)
	)
)


(:derived (inferred-HasQAestimation ?x ?y) 
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


(:derived (inferred-IsQAtype ?x ?y) 
	(and
		(isQAtype ?x ?y)
	)
)


(:derived (inferred-QAvalue ?x) 
	(and
		(QAvalue ?x)
	)
)


(:derived (inferred-QAvalue ?x) 
	(exists (?y)
 		(and
			(inferred-IsQAtype ?x ?y)
		)
	)
 )


(:derived (inferred-QAvalue ?x) 
	(exists (?y)
 		(and
			(inferred-Qa_has_value ?x ?y)
		)
	)
 )


(:derived (inferred-QAvalue ?y) 
	(exists (?x)
 		(and
			(inferred-HasQAestimation ?x ?y)
		)
	)
 )


(:derived (inferred-Qa_has_value ?x ?y) 
	(and
		(qa_has_value ?x ?y)
	)
)


(:derived (inferred-QualityAttributeType ?x) 
	(and
		(QualityAttributeType ?x)
	)
)


(:derived (inferred-QualityAttributeType ?y) 
	(exists (?x)
 		(and
			(inferred-IsQAtype ?x ?y)
		)
	)
 )


(:derived (inferred-RequiresC ?x ?y) 
	(and
		(requiresC ?x ?y)
	)
)


(:derived (inferred-RequiresF ?x ?y) 
	(and
		(requiresF ?x ?y)
	)
)


(:derived (inferred-SameAs ?x ?y) 
	(and
		(sameAs ?x ?y)
	)
)


(:derived (inferred-SolvesF ?fd_unground ?f) 
	(and
		(= ?fd_unground fd_unground)
		(and
			(inferred-Function ?f)
			(inferred-FunctionDesign fd_unground)
		)
	)
 )


(:derived (inferred-SolvesF ?x ?y) 
	(and
		(solvesF ?x ?y)
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

      (or (= ?fd_goal fd_unground)
        (not
          (exists (?fd)
            (and
              (inferred-SolvesF ?fd ?f)
              (not (inferred-Fd_realisability ?fd false_boolean))
              (inferred-FdBetterUtility  ?fd ?fd_goal)
            )
          )
        )
      )
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
      (or (= ?fd_goal fd_unground)
        (not
          (exists (?fd)
            (and
              (inferred-SolvesF ?fd ?f)
              (not (inferred-Fd_realisability ?fd false_boolean))
              (inferred-FdBetterUtility  ?fd ?fd_goal)
            )
          )
        )
      )
    )
    :effect (and
      (functionGrounding ?f ?fd_goal)
    )
  )

  (:action move
    :parameters (?wp1 ?wp2 - waypoint)
    :precondition (and
      (robot_at ?wp1)
      (corridor ?wp1 ?wp2)
      (exists (?a ?f1 ?fd1)
        (and
          (inferred-Action ?a)
          (= ?a a_move)
          (inferred-requiresF ?a ?f1)
          (inferred-F_active ?f1 true_boolean)
        )
      )
    )
    :effect (and
      (not(robot_at ?wp1))
      (robot_at ?wp2)
    )
  )

)
