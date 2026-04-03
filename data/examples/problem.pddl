; AV Planning Problem: Navigate from intersection to destination
; with safety constraints

(define (problem av-route-001)
  (:domain av-planning)

  (:objects
    vehicle_1 - vehicle
    loc_start loc_intersection loc_crosswalk loc_destination - location
    lane_1 lane_2 - lane
  )

  (:init
    (at vehicle_1 loc_start)
    (connected loc_start loc_intersection)
    (connected loc_intersection loc_crosswalk)
    (connected loc_crosswalk loc_destination)
    (speed-low vehicle_1)
    (lane-clear lane_1)
    (lane-clear lane_2)
  )

  (:goal
    (and
      (at-destination vehicle_1)
      (not (collision vehicle_1))
    )
  )

  ; Custom safety invariants (AV extension)
  (:safety-invariants
    (no-collision must-not-hold collision)
  )
)
