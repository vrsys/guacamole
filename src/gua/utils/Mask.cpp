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

// class header
#include <gua/utils/Mask.hpp>

// guacamole headers
#include <gua/platform.hpp>
#include <gua/utils/logger.hpp>

// external headers
#include <iostream>

namespace gua {

////////////////////////////////////////////////////////////////////////////////

Mask::Mask(std::string const& render_mask)
    : expression_(render_mask) {}

////////////////////////////////////////////////////////////////////////////////

bool Mask::check(std::set<std::string> const& groups) const {

    return expression_.check(groups);
}

////////////////////////////////////////////////////////////////////////////////

Mask::BasicExpression::BasicExpression(std::string const& expression)
    : type_(VALUE), children_(), value_("") {

    int open_brackets(0);
    std::string curr_expr;
    std::string expr(remove_useless_brackets(expression));
    std::vector<std::string> child_expressions;

    if (expr == "")
        return;

    for (unsigned i(0); i < expr.size(); ++i) {
        char c(expr[i]);

        switch (c) {
            case ' ': {
                // ignore whitespaces
                break;
            }
            case '(': {
                ++open_brackets;

                curr_expr += '(';

                break;
            }
            case ')': {
                --open_brackets;

                if (open_brackets < 0) {
                    WARNING("Failed to parse expression %s: "
                            "Unexpected ) at %u",
                            expr.c_str(),
                            i);
                    return;
                } else if (open_brackets == 0 && curr_expr != "") {
                    if (curr_expr[curr_expr.size() - 1] == '(' ||
                        curr_expr[curr_expr.size() - 1] == '!') {

                        WARNING("Failed to parse expression %s: "
                                "Unexpected ) at %u",
                                expr.c_str(),
                                i);
                        return;
                    }

                    child_expressions.push_back(curr_expr + ')');
                    curr_expr = "";
                } else if (open_brackets > 0) {
                    curr_expr += ')';
                }
                break;
            }
            case '&': {

                if (child_expressions.size() == 0 && curr_expr == "") {
                    WARNING("Failed to parse expression %s: "
                            "Unexpected & at %u",
                            expr.c_str(),
                            i);
                    return;
                } else if (open_brackets == 0) {
                    if (curr_expr != "") {
                        child_expressions.push_back(curr_expr);
                        curr_expr = "";
                    }

                    if (type_ == OR) {
                        WARNING("Failed to parse expression %s: "
                                "Unexpected & at %u. Don't mix & "
                                "and | in one expression. Please "
                                "use brackets!",
                                expr.c_str(),
                                i);
                        return;
                    }

                    type_ = AND;
                } else if (open_brackets > 0) {
                    curr_expr += '&';
                }

                break;
            }
            case '|': {

                if (child_expressions.size() == 0 && curr_expr == "") {
                    WARNING("Failed to parse expression %s: "
                            "Unexpected | at %u",
                            expr.c_str(),
                            i);
                    return;
                } else if (open_brackets == 0) {
                    if (curr_expr != "") {
                        child_expressions.push_back(curr_expr);
                        curr_expr = "";
                    }

                    if (type_ == AND) {
                        WARNING("Failed to parse expression %s: "
                                "Unexpected | at %u. Don't mix & "
                                "and | in one expression. Please "
                                "use brackets!",
                                expr.c_str(),
                                i);
                        return;
                    }

                    type_ = OR;
                } else if (open_brackets > 0) {
                    curr_expr += '|';
                }

                break;
            }
            case '!': {

                if (open_brackets == 0 && curr_expr == "") {
                    curr_expr += '!';
                } else if (open_brackets <= 0) {
                    WARNING("Failed to parse expression %s: "
                            "Unexpected ! at %u.",
                            expr.c_str(),
                            i);
                    return;
                } else if (open_brackets > 0) {
                    curr_expr += '!';
                }

                break;
            }
            default: {
                curr_expr += c;

                break;
            }
        }
    }

    if (open_brackets > 0) {
        WARNING("Failed to parse expression %s: "
                "Expected ) at end of input.",
                expr.c_str());
        return;
    }

    if (curr_expr != "")
        child_expressions.push_back(curr_expr);

    if (child_expressions.size() == 1 && child_expressions[0][0] == '!') {
        if (child_expressions[0].size() == 1) {
            WARNING("Failed to parse expression %s.", expr.c_str());
            return;
        }

        type_ = NOT;
        children_.push_back(BasicExpression(child_expressions[0].substr(1)));

        return;
    }

    if (type_ == VALUE) {
        value_ = *child_expressions.begin();
        return;
    }

    for (auto& child : child_expressions) {
        children_.push_back(BasicExpression(child));
    }

}

////////////////////////////////////////////////////////////////////////////////

bool Mask::BasicExpression::check(std::set<std::string> const &
                                        groups) const {

    switch (type_) {
        case AND: {
            for (auto& child : children_) {
                if (!child.check(groups))
                    return false;
            }
            return true;
        }
    case OR: {
            for (auto& child : children_) {
                if (child.check(groups))
                    return true;
            }
            return false;
        }
    case NOT: { return !children_.begin()->check(groups); }
    case VALUE: {
            if (value_ == "")
                return true;
            if (groups.size() == 0)
                return false;
            return groups.find(value_) != groups.end();
        }
    }
    return false;
}

////////////////////////////////////////////////////////////////////////////////

std::string Mask::BasicExpression::remove_useless_brackets(
    std::string const & input) const {

    // remove leading and trailing corresponding brackets
    unsigned lead_brackets(0);
    while (lead_brackets < input.size() && input[lead_brackets] == '(') {
        ++lead_brackets;
    }

    // count removable brackets
    if (lead_brackets > 0) {
        unsigned del_brackets(lead_brackets);
        unsigned open_brackets(lead_brackets);

        for (unsigned i(lead_brackets); i < input.size() - lead_brackets; ++i) {
            if (input[i] == '(') {
                ++open_brackets;
            } else if (input[i] == ')') {
                --open_brackets;
                if (open_brackets < del_brackets)
                    del_brackets = open_brackets;
            }
        }

        return input.substr(del_brackets, input.size() - 2 * del_brackets);

    }

    return input;
}

////////////////////////////////////////////////////////////////////////////////

}
