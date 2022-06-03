#pragma once 

#define FLOCK_SIZE	        5		// Number of robots in flock

/* REYNOLDS */
#define RULE1_THRESHOLD     0.20   		// Threshold to activate aggregation rule. default 0.20
#define RULE1_WEIGHT        0.6/10	    // Weight of aggregation rule. default 0.6/10
#define RULE2_THRESHOLD     0.15   		// Threshold to activate dispersion rule. default 0.15
#define RULE2_WEIGHT        0.02/10 	// Weight of dispersion rule. default 0.02/10
#define RULE3_WEIGHT        1.0/10 	    // Weight of alignment rule. default 1.0/10
#define MIGRATION_WEIGHT    0.5/10 	    // Wheight of attraction towards the common goal. default 0.5/10
#define MIGRATORY_URGE 		1 			// Tells the robots if they should just go forward or move towards a specific migratory direction

#define NEIGHBOURHOOD 		0 	// Tells the robot considering neighbors or all robots during flocking
#define NEIGH_THRESHOLD 	0.4 // Threshold to consider neighbourhood
#define INTER_VEHICLE_COM 	0 	// Set 1 if there is intervehicle communication (not supported here)

