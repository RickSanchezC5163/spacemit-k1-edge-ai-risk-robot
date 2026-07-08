# Hazard Scene Simulation Plan - 2026-06-25

Goal: create safe, repeatable constrained-space test scenes for the non-arm
risk pipeline.

## Scene 1: Low-light Passage

Setup:

- dim room or shaded corridor
- boxes or chairs forming a narrow passage
- robot lamp initially off

Expected event:

- `low_light`
- recommended action: `turn_on_light_and_recheck`

Safety:

- do not run lamp at full power for long
- monitor battery and lamp temperature

## Scene 2: Soft Obstacle

Setup:

- cloth strip, foam block, plastic bag, or small paper box
- placed 0.5-1.5 m in front of robot

Expected event:

- `soft_obstacle`
- medium risk if close

Safety:

- use lightweight materials only

## Scene 3: Hard Obstacle / Blocked Path

Setup:

- large cardboard box, rigid crate, or chair legs
- blocks the path visually and in lidar

Expected events:

- `hard_obstacle`
- `blocked_path`

Safety:

- keep speeds low
- no automatic motion in this stage

## Scene 4: Cable Or Wire Risk

Setup:

- soft rope, unplugged cable, or yarn line
- laid across the passage

Expected event:

- `cable_or_wire`

Safety:

- do not let it touch live power
- do not run the robot over it until manual safety is confirmed

## Scene 5: Reflective / Occlusion Interference

Setup:

- reflective film, shiny plastic, mirror-like packaging, or partial board cover
- place near an object edge

Expected event:

- `reflective_noise`

Safety:

- avoid lasers or intense directed light
- use safe reflective film or packaging only

## General Constraints

- no real dangerous materials
- no real water near electronics
- simulate water with blue plastic film or reflective film
- every scene must be safe to assemble in a lab or dorm room
- one person should be ready to cut power during chassis tests
