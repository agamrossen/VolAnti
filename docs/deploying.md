# Deploying

## What to expect for range

Range moves by an order of magnitude with wind, so a figure without its wind speed is meaningless. These are for a full 7 to 9 in multirotor under load, unit face up in the open with grille cloth fitted.

| Conditions | Wind | Expect detection at |
|---|---|---|
| Still air, quiet rural site | under 2 m/s, leaves still | 100 to 200 m |
| Light breeze, ordinary background | 2 to 5 m/s, leaves moving | 50 to 120 m |
| Moderate wind, or a noisy site | 5 to 8 m/s, branches moving | 15 to 50 m |
| Strong wind | over 8 m/s | tens of metres at best |

The 104 m measurement was made in light wind on a noisy street, so it sits where the second row says it should. Warning time is detection distance divided by closing speed. At 27 m/s, 120 m is about 4.5 s and 50 m is under 2 s.

A steady distant hover takes longer to alarm than an approach. Loud steady noise near the unit shortens range without causing false alarms. Your own drone triggers it. Close conversation can trigger the comb tiers, which is rare in an outdoor mounting position and known.

## Several units

Every unit runs the full detector and alerts on its own evidence. There is no voting. The radio spreads an alert, it does not confirm one.

| Role | Where | Power |
|---|---|---|
| Listening units | Across the approach sides, spaced so coverage touches | Solar or battery |
| Indoor units | Where people are, mostly repeating the alert | Mains |
| Cold spare | On a shelf, assembled and flashed | |

Warning time = (detection distance + standoff) ÷ threat speed. A unit that reliably hears an aircraft at 100 m, sited 300 m out, gives about 15 seconds against something moving at 27 m/s.

## Siting

Face up, parallel to the ground. The array is omnidirectional at drone rates, so the only orientation that matters is the unit's own body shadowing the sky. In shade: sun cooks the cell and ages the e-paper. Two to four metres up, above head height and below rooflines. A few metres clear of air conditioning, generators and transformers. They will not false-alarm it, but they raise the floor and cost range.

Do not publish where your units are.

## Radio region

The LoRa frequency has no universal default. Set it for your country before transmitting, and set every unit at a site the same.

| Region | Band |
|---|---|
| UK and EU | 868 MHz |
| Israel | 917 to 920 MHz, verify the current allocation |
| US | 915 MHz |

The Ra-01H covers all of them. Link settings are SF9, BW125, CR4/5.

## Mains units

The USB input is the charger and the battery stays fitted. It is the buffer that keeps the rail up when beeper, motor and radio fire together. Use a locally certified 5 V supply, a braided USB-A to USB-C cable no longer than 2 m at 22 AWG, a drip loop under the port with self-amalgamating tape over the connector, and an RCD on the circuit.

## Service

| When | What |
|---|---|
| Weekly | Green blink present, screen sane, note the alert count |
| 3 months | Open one mains unit and look at the cell for swelling |
| 6 months | Replace cells. Check the silica sachets. A saturated one means that unit's seal has failed. |
| After any alert | Pull the event log and keep it. Real events are the rarest data this project has. |

## Drills

Decide in advance what an alert means, who moves where, who checks and who stands down, and walk it once with the beeper going. Seconds of warning only help people who already know what to do with them.
