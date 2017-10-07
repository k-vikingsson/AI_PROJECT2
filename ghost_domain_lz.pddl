(define (domain ghost)
    (:requirements :typing :conditional-effects)
    (:types place)
    (:predicates (connected ?x ?y - place)  ;;wheather the location is connected
	             (at ?x - place)            ;;current location for pacman. 
	             (pacmanAt ?x - place)       ;;pacmans' location
    )
	
    (:action move
        :parameters (?curpos ?nextpos - place)
        :precondition (and (at ?curpos) 
                           (connected ?curpos ?nextpos)
                      )
                       
        :effect (and (at ?nextpos) 
                     (not (at ?curpos))
                     (not (pacmanAt ?nextpos))
                     )))
                 
    
