/**
 * @addtogroup wr_control_drive_arm
 * @{
 */

/**
 * @file
 * @author Ben Nowotny
 * @brief Defines a representation for the channels of a roboclaw controller
 *
 */

#ifndef ROBOCLAW_CHANNEL_H
#define ROBOCLAW_CHANNEL_H

/**
 * @brief Represents the two channels for a wroboclaw controller
 *
 */
enum class RoboclawChannel {
    /// Analogous to 'left', the 'first channel'
    A,
    /// Analogous to 'right', the 'second channel'
    B
};

#endif

/// @}
