/*
 * Copyright (C) 2019-2020 makr@makr.zone
 *
 * This 3D printed feeder is inteded to be used together with the "BlindsFeeder" feeder in OpenPNP. 
 *
 * This file is part of OpenPnP. 
 *
 * OpenPnP is free software: you can redistribute it and/or modify it under the terms of the GNU
 * General Public License as published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 * 
 * OpenPnP is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
 * the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License along with OpenPnP. If not, see
 * <http://www.gnu.org/licenses/>.
 * 
 * For more information about OpenPnP visit http://openpnp.org
 */
 
 debug_view = true;
 label_view = false;

// Include the BlindsFeeder Library. 
// To learn the details about available parameters, please read the extensive comments in the library file. 
// The library also defines some 3D printing constants that you might want to tune along with some advice 
// how to setup your slicer. 
include <BlindsFeeder-Library.scad>

// All dimensions in Millimeters.

// Let's create some tape definitions
// Again, please read the extensive commentPocket pitchs in the library file to learn what 
// parameters are avaialable and what exactly they mean and do. 
tape0402 = TapeDefinition(
    tape_width=8-0.9, // smd
    tape_thickness=5.6,
    // Pitch of the pockets in the tape. Use 2mm or multiples of 4mm.
    pocket_pitch=2,  // Taschenplatz
    // Pocket width across the tape.
    pocket_width=2.4,
    // Tape play. Use this to adjust the precise tape width and the play for print imperfections.
    tape_play=-0.03,
    // Horizontal per-side play of the cover to allow for imperfections of the print.
    cover_play=0.08);

tape0401 = TapeDefinition( // für schmalle tapes
    tape_width=24-0.8,
    tape_thickness=5.6,
    // Pitch of the pockets in the tape. Use 2mm or multiples of 4mm.
    pocket_pitch=2,  // Taschenplatz
    // Pocket width across the tape.
    pocket_width=2.4,
    // Tape play. Use this to adjust the precise tape width and the play for print imperfections.
    tape_play=-0.03,
    // Horizontal per-side play of the cover to allow for imperfections of the print.
    cover_play=0.08);

tape0403 = TapeDefinition( // für schmalle tapes
    tape_width=8-0.8,
    tape_thickness=5.6,
    // Pitch of the pockets in the tape. Use 2mm or multiples of 4mm.
    pocket_pitch=2,  // Taschenplatz
    // Pocket width across the tape.
    pocket_width=2.4,
    // Tape play. Use this to adjust the precise tape width and the play for print imperfections.
    tape_play=-0.03,
    // Horizontal per-side play of the cover to allow for imperfections of the print.
    cover_play=0.08);

tape0404 = TapeDefinition( // für schmalle tapes
    tape_width=12-0.8,
    tape_thickness=5.6,
    // Pitch of the pockets in the tape. Use 2mm or multiples of 4mm.
    pocket_pitch=2,  // Taschenplatz
    // Pocket width across the tape.
    pocket_width=2.4,
    // Tape play. Use this to adjust the precise tape width and the play for print imperfections.
    tape_play=-0.03,
    // Horizontal per-side play of the cover to allow for imperfections of the print.
    cover_play=0.08);


// Create the feeder array with these tape definitions.
// Note the BlindsFeeder has a myriad of parameters you can tweak, the ones used here are just the most important. 
// See the Library file to learn more. 
rotate([0, 0, 180]) BlindsFeeder(
    // Tape length from feeder edge to edge (not including the margin), usually multiples of 4mm. 
    // Other values are supported if you manually adjust the default 2mm edge distance in the OpenPNP feeder.
    tape_length=40,
    
    // For OCR, add a margin at the begin of the tape.
    margin_length_begin=16,
    
    // Define the lanes with number, tape definitinon, part label (String array with multiple lines).
    arrayed_tape_lanes=      [
        LaneDefinition(1, tape0402,   ["C269541"]), 
        LaneDefinition(1, tape0401,   ["USB-C"]), 
        LaneDefinition(1, tape0403,   ["C15643"]), 
        LaneDefinition(1, tape0404,   ["C7813"]), 
        LaneDefinition(1, tape0403,   ["C264432"]), 
        LaneDefinition(1, tape0402,   ["C282732"]), 
        ],
    
    label=label_view,
    tray=(! label_view),
  //  cover=(! label_view),
    debug=debug_view
);

//difference(){
//translate([-20, 0, 0]) cube([14, 14, 2]);
//translate([-13, 7, 0]) cylinder(5.3, 2.65, 3);
//}

