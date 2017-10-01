(define (domain ghost)
    (:requirements :typing :conditional-effects)
    (:types        place - object)
    (:predicates (connected ?x ?y - place)  ;;wheather the location is connected
	             (at ?x - place)            ;;current location for ghost.
	             (visited ?x - place)       ;;recording visited locations
	             (pacmanAt ?x - place)      ;;pacmans' location
	             (superPacman)              ;;wheather pacman is super of not.
    )
	
    (:action move
        :parameters (?curpos ?nextpos - place)
        :precondition (and (at ?curpos) 
                           (connected ?curpos ?nextpos)
                       )
        :effect (and (at-robot ?nextpos) 
                     (not (at ?curpos))
                     ;;eat pacman only when the pacman is not powered.
                     (when(not (superPacman)) (not (pacmanAt ?nextpos)))
                     (visited ?nextpos)
                 )
    )

)
