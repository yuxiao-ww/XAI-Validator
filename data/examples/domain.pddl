; Autonomous Vehicle Planning Domain
; Used for XAI validation research at WashU

(define (domain av-planning)
  (:requirements :strips :typing)

  (:types vehicle location lane)

  (:predicates
    (at ?v - vehicle ?l - location)
    (connected ?from - location ?to - location)
    (lane-clear ?ln - lane)
    (speed-high ?v - vehicle)
    (speed-low ?v - vehicle)
    (collision ?v - vehicle)
    (at-destination ?v - vehicle)
    (obstacle-ahead ?v - vehicle)
    (traffic-light-red ?l - location)
    (pedestrian-crossing ?l - location)
  )

  ; Move vehicle from one location to another
  (:action move
    :parameters (?v - vehicle ?from - location ?to - location)
    :precondition (and
      (at ?v ?from)
      (connected ?from ?to)
      (not (traffic-light-red ?to))
      (not (obstacle-ahead ?v))
    )
    :effect (and
      (at ?v ?to)
      (not (at ?v ?from))
    )
  )

  ; Accelerate vehicle
  (:action accelerate
    :parameters (?v - vehicle)
    :precondition (and
      (speed-low ?v)
      (not (obstacle-ahead ?v))
      (not (pedestrian-crossing))
    )
    :effect (and
      (speed-high ?v)
      (not (speed-low ?v))
    )
  )

  ; Decelerate vehicle (safe braking)
  (:action decelerate
    :parameters (?v - vehicle)
    :precondition (speed-high ?v)
    :effect (and
      (speed-low ?v)
      (not (speed-high ?v))
    )
  )

  ; Emergency stop
  (:action emergency-stop
    :parameters (?v - vehicle)
    :precondition (obstacle-ahead ?v)
    :effect (and
      (speed-low ?v)
      (not (speed-high ?v))
    )
  )

  ; Change lane
  (:action change-lane
    :parameters (?v - vehicle ?ln - lane)
    :precondition (and
      (lane-clear ?ln)
      (speed-low ?v)
    )
    :effect (lane-clear ?ln)
  )

  ; Arrive at destination
  (:action arrive
    :parameters (?v - vehicle ?dest - location)
    :precondition (and
      (at ?v ?dest)
      (speed-low ?v)
    )
    :effect (at-destination ?v)
  )
)
