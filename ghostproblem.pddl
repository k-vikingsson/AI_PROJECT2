﻿;; COMP90054 AI Planning for Autonomy
;; Project 2
;; Filename: ghostproblem.pddl
;;
;; Team Name: PacmanNull
;; Members:   Marco Liu     696672
;;            Xiangyang Zhu 715949
;;            Zequn Ma      696586
;;
(define (problem grid-10)
(:domain ghost)
(:objects 
    loc-x0-y0
    loc-x0-y1
    loc-x0-y2
    loc-x0-y3
    loc-x0-y4
    loc-x0-y5
    loc-x1-y0
    loc-x1-y1
    loc-x1-y2
    loc-x1-y3
    loc-x1-y4
    loc-x1-y5
    loc-x2-y0
    loc-x2-y1
    loc-x2-y2
    loc-x2-y3
    loc-x2-y4
    loc-x2-y5
    loc-x3-y0
    loc-x3-y1
    loc-x3-y2
    loc-x3-y3
    loc-x3-y4
    loc-x3-y5
    loc-x4-y0
    loc-x4-y1
    loc-x4-y2
    loc-x4-y3
    loc-x4-y4
    loc-x4-y5
    loc-x5-y0
    loc-x5-y1
    loc-x5-y2
    loc-x5-y3
    loc-x5-y4
    loc-x5-y5
- place 
        
)
(:init
    (at loc-x0-y0)
    (pacmanat loc-x4-y5)
    (connected loc-x0-y0 loc-x0-y1)
    (connected loc-x0-y1 loc-x0-y0)
    (connected loc-x0-y1 loc-x0-y2)
    (connected loc-x0-y2 loc-x0-y1)
    (connected loc-x0-y2 loc-x0-y3)
    (connected loc-x0-y3 loc-x1-y3)
    (connected loc-x0-y3 loc-x0-y2)
    (connected loc-x0-y3 loc-x0-y4)
    (connected loc-x0-y4 loc-x1-y4)
    (connected loc-x0-y4 loc-x0-y3)
    (connected loc-x0-y4 loc-x0-y5)
    (connected loc-x0-y5 loc-x1-y5)
    (connected loc-x0-y5 loc-x0-y4)

    (connected loc-x1-y3 loc-x0-y3)
    (connected loc-x1-y3 loc-x2-y3)
    (connected loc-x1-y3 loc-x1-y4)
    (connected loc-x1-y4 loc-x0-y4)
    (connected loc-x1-y4 loc-x2-y4)
    (connected loc-x1-y4 loc-x1-y3)
    (connected loc-x1-y4 loc-x1-y5)
    (connected loc-x1-y5 loc-x0-y5)
    (connected loc-x1-y5 loc-x2-y5)
    (connected loc-x1-y5 loc-x1-y4)
    
    (connected loc-x2-y0 loc-x3-y0)
    (connected loc-x2-y0 loc-x2-y1)
    (connected loc-x2-y1 loc-x2-y0)
    (connected loc-x2-y1 loc-x2-y2)
    (connected loc-x2-y2 loc-x2-y1)
    (connected loc-x2-y2 loc-x2-y3)
    (connected loc-x2-y3 loc-x1-y3)
    (connected loc-x2-y3 loc-x3-y3)
    (connected loc-x2-y3 loc-x2-y2)
    (connected loc-x2-y3 loc-x2-y4)
    (connected loc-x2-y4 loc-x1-y4)
    (connected loc-x2-y4 loc-x2-y3)
    (connected loc-x2-y4 loc-x2-y5)
    (connected loc-x2-y5 loc-x1-y5)
    (connected loc-x2-y5 loc-x2-y4)
    
    (connected loc-x3-y0 loc-x2-y0)
    (connected loc-x3-y0 loc-x4-y0)
    (connected loc-x3-y3 loc-x2-y3)
    (connected loc-x3-y3 loc-x4-y3)
    
    (connected loc-x4-y0 loc-x3-y0)
    (connected loc-x4-y0 loc-x5-y0)
    (connected loc-x4-y2 loc-x5-y2)
    (connected loc-x4-y2 loc-x4-y3)
    (connected loc-x4-y3 loc-x3-y3)
    (connected loc-x4-y3 loc-x4-y2)
    (connected loc-x4-y3 loc-x4-y4)
    (connected loc-x4-y4 loc-x5-y4)
    (connected loc-x4-y4 loc-x4-y3)
    (connected loc-x4-y4 loc-x4-y5)
    (connected loc-x4-y5 loc-x5-y5)
    (connected loc-x4-y5 loc-x4-y4)
    
    (connected loc-x5-y0 loc-x4-y0)
    (connected loc-x5-y0 loc-x5-y1)
    (connected loc-x5-y1 loc-x5-y0)
    (connected loc-x5-y1 loc-x5-y2)
    (connected loc-x5-y2 loc-x4-y2)
    (connected loc-x5-y2 loc-x5-y1)
    (connected loc-x5-y4 loc-x4-y4)
    (connected loc-x5-y4 loc-x5-y5)
    (connected loc-x5-y5 loc-x4-y5)
    (connected loc-x5-y5 loc-x5-y4)
)
(:goal
(not (or 
    (pacmanat loc-x0-y0)
    (pacmanat loc-x0-y1)
    (pacmanat loc-x0-y2)
    (pacmanat loc-x0-y3)
    (pacmanat loc-x0-y4)
    (pacmanat loc-x0-y5)
    
    (pacmanat loc-x1-y0)
    (pacmanat loc-x1-y1)
    (pacmanat loc-x1-y2)
    (pacmanat loc-x1-y3)
    (pacmanat loc-x1-y4)
    (pacmanat loc-x1-y5)
    
    (pacmanat loc-x2-y0)
    (pacmanat loc-x2-y1)
    (pacmanat loc-x2-y2)
    (pacmanat loc-x2-y3)
    (pacmanat loc-x2-y4)
    (pacmanat loc-x2-y5)
    
    (pacmanat loc-x3-y0)
    (pacmanat loc-x3-y1)
    (pacmanat loc-x3-y2)
    (pacmanat loc-x3-y3)
    (pacmanat loc-x3-y4)
    (pacmanat loc-x3-y5)
   
    (pacmanat loc-x4-y0)
    (pacmanat loc-x4-y1)
    (pacmanat loc-x4-y2)
    (pacmanat loc-x4-y3)
    (pacmanat loc-x4-y4)
    (pacmanat loc-x4-y5)
    
    (pacmanat loc-x5-y0)
    (pacmanat loc-x5-y1)
    (pacmanat loc-x5-y2)
    (pacmanat loc-x5-y3)
    (pacmanat loc-x5-y4)
    (pacmanat loc-x5-y5)
)
)
)
)


