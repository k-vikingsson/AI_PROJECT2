(define (domain pacman)
    (:requirements :typing :conditional-effects)
    (:types        place - object)
    (:predicates (connected ?x ?y - place)  ;;wheather the location is connected
	             (at ?x - place)            ;;current location for pacman.
	             (visited ?x - place)       ;;recording visited location
	             (foodAt ?x - place)        ;;food's location
	             (ghostAt ?x - place)       ;;pacmans' location
	             (capsulesAt ?x - place)    ;;capsules' location
	             (powered)                  ;;wheather pacman is powered of not.
    )
	
    (:action move
        :parameters (?curpos ?nextpos - place powered)
        :precondition (and (at ?curpos) 
                           (connected ?curpos ?nextpos)
                       )
        :effect (and (at ?nextpos) 
                     (not (at ?curpos))
                     (not (foodAt ?nextpos))
                     (not (capsulesAt ?nextpos))
                     ;;can eat ghost only when the pacman is powered.
                     (when(powered) (not (ghostAt ?nextpos)))
                     (visited ?nextpos)
                 )
    )

)
