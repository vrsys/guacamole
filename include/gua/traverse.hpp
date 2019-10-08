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

#ifndef GUA_NODE_TRAVERSER_HPP
#define GUA_NODE_TRAVERSER_HPP

#include <gua/node/Node.hpp>

namespace gua
{
template <class UnaryOperation, class UnaryPredicate>
void dfs_traverse_if(node::Node* node, UnaryOperation unary_op, UnaryPredicate pred)
{
    if(pred(node))
    {
        unary_op(node);
        for(auto& c : node->get_children())
        {
            dfs_traverse_if(c.get(), unary_op, pred);
        }
    }
}

template <class UnaryOperation>
void dfs_traverse(node::Node* node, UnaryOperation unary_op)
{
    unary_op(node);
    for(auto& c : node->get_children())
    {
        dfs_traverse(c.get(), unary_op);
    }
}

} // namespace gua

#endif // GUA_NODE_TRAVERSER_HPP
