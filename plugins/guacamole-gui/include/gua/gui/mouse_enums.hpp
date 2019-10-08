/******************************************************************************
 * guacamole - delicious VR                                                   *
 *                                                                            *
 * Copyright: (c) 2011-2013 Bauhaus-Universität Weimar                        *
 * Contact:   felix.lauer@uni-weimar.de / simon.schneegans@uni-weimar.de      *
 *                                                                            *
 * This program is free software: you can redistribute it and/or modify it    *
 * under the terms of the GNU General Public License as published by the Free *
 * Software Foundation, either version 3 of the License, or (at your option)  *
 * any later version.                                                         *
 *                                                                            *
 * This program is distributed in the hope that it will be useful, but        *
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY *
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License   *
 * for more details.                                                          *
 *                                                                            *
 * You should have received a copy of the GNU General Public License along    *
 * with this program. If not, see <http://www.gnu.org/licenses/>.             *
 *                                                                            *
 ******************************************************************************/

#ifndef GUA_GUI_MOUSE_ENUMS_HPP
#define GUA_GUI_MOUSE_ENUMS_HPP

namespace gua
{
enum class Button : int
{
    BUTTON_1 = 0,
    BUTTON_2 = 1,
    BUTTON_3 = 2,
    BUTTON_4 = 3,
    BUTTON_5 = 4,
    BUTTON_6 = 5,
    BUTTON_7 = 6,
    BUTTON_8 = 7
};

enum class Cursor : int
{
    POINTER,
    CROSS,
    HAND,
    IBEAM,
    WAIT,
    HELP,
    EAST_RESIZE,
    NORTH_RESIZE,
    NORTH_EAST_RESIZE,
    NORTH_WEST_RESIZE,
    SOUTH_RESIZE,
    SOUTH_EAST_RESIZE,
    SOUTH_WEST_RESIZE,
    WEST_RESIZE,
    NORTH_SOUTH_RESIZE,
    EAST_WEST_RESIZE,
    NORTH_EAST_SOUTH_WEST_RESIZE,
    NORTH_WEST_SOUTH_EAST_RESIZE,
    COLUMN_RESIZE,
    ROW_RESIZE,
    MIDDLE_PANNING,
    EAST_PANNING,
    NORTH__PANNING,
    NORTH_EAST_PANNING,
    NORTH_WEST_PANNING,
    SOUTH__PANNING,
    SOUTH_EAST_PANNING,
    SOUTH_WEST_PANNING,
    WEST_PANNING,
    MOVE,
    VERTICAL_TEXT,
    CELL,
    CONTEXT_MENU,
    ALIAS,
    PROGRESS,
    NO_DROP,
    COPY,
    NONE,
    NOT_ALLOWED,
    ZOOM_IN,
    ZOOM_OUT,
    GRAB,
    GRABBING,
    CUSTOM
};

} // namespace gua

#endif // GUA_GUI_MOUSE_ENUMS_HPP
