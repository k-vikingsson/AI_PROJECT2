(define (domain pacman)
    (:requirements :typing :conditional-effects)
    (:types place)
    (:predicates (connected ?x ?y - place)  ;;wheather the location is connected
	             (at ?x - place)            ;;current location for pacman. 
	             (foodAt ?x - place)        ;;food's location
	             (ghostAt ?x - place)       ;;pacmans' location
	             (capsulesAt ?x - place)    ;;capsules' location
	             (powered)                  ;;wheather pacman is powered of not.
    )
	
    (:action move
        :parameters (?curpos ?nextpos - place)
        :precondition (and (at ?curpos) 
                           (connected ?curpos ?nextpos)
                           (or (and (not (powered)) (not (ghostAt ?nextpos)) )
                                    (powered)
                            )
                      )
                       
        :effect (and (at ?nextpos) 
                     (not (at ?curpos))
                     (not (foodAt ?nextpos))
                     (not (capsulesAt ?nextpos))
                     (when (capsulesAt ?curpos) (powered))
                     )))
                 
    

