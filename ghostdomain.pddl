;; COMP90054 AI Planning for Autonomy
;; Project 2
;; Filename: ghostdomain.pddl
;;
;; Team Name: PacmanNull
;; Members:   Marco Liu     696672
;;            Xiangyang Zhu 715949
;;            Zequn Ma      696586
;;
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
                 
    