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
#include <gua/renderer/NURBSShader.hpp>

// guacamole headers

// external headers
#include <sstream>

namespace gua {

////////////////////////////////////////////////////////////////////////////////

NURBSShader::
NURBSShader()
{}

////////////////////////////////////////////////////////////////////////////////

NURBSShader::
~NURBSShader()
{}

////////////////////////////////////////////////////////////////////////////////
std::string const
NURBSShader::surface_horner_evaluation ()
{
  return horner_simple () + horner_derivatives ();
}

////////////////////////////////////////////////////////////////////////////////
std::string const
NURBSShader::horner_simple ()
{
    return std::string("                                                                                                        \n\
        /*******************************************************************************                                        \n\
        * Evaluate Surface using modificated horner algorithm in Bernstein basis                                                \n\
        * points assumed in homogenous coordinates! :  p = [wx wy wz w]                                                         \n\
        *******************************************************************************/                                        \n\
        void                                                                                                                    \n\
        evaluateSurface(in samplerBuffer  data,                                                                                 \n\
                        in int            index,                                                                                \n\
                        in int            orderU,                                                                               \n\
                        in int            orderV,                                                                               \n\
                        in vec2           uv,                                                                                   \n\
                        out vec4          p)                                                                                    \n\
        {                                                                                                                       \n\
          p = vec4(0.0);                                                                                                        \n\
                                                                                                                                \n\
          float bcu = 1.0;                                                                                                      \n\
          float un = 1.0;                                                                                                       \n\
          int deg_u = orderU - 1;                                                                                               \n\
          int deg_v = orderV - 1;                                                                                               \n\
                                                                                                                                \n\
          vec4 u0_0 = texelFetchBuffer(data, index              ) * (1.0 - uv[0]);                                              \n\
          vec4 u0_1 = texelFetchBuffer(data, index + 1          ) * (1.0 - uv[0]);                                              \n\
          vec4 u1_0 = texelFetchBuffer(data, index + orderU    ) * (1.0 - uv[0]);                                               \n\
          vec4 u1_1 = texelFetchBuffer(data, index + orderU + 1) * (1.0 - uv[0]);                                               \n\
                                                                                                                                \n\
          /**************************************** 1. step : horner for first 2 rows *********************************/        \n\
          int i;                                                                                                                \n\
          for (i = 1; i <= deg_u - 2; ++i) {                                                                                    \n\
            un = un * uv[0];                                                                                                    \n\
            bcu = bcu * (float(deg_u - i) / float(i));                                                                          \n\
                                                                                                                                \n\
            u0_0 = (u0_0 + un * bcu * texelFetchBuffer(data, index + i    ))           * (1.0 - uv[0]);                         \n\
            u0_1 = (u0_1 + un * bcu * texelFetchBuffer(data, index + i + 1))           * (1.0 - uv[0]);                         \n\
                                                                                                                                \n\
            u1_0 = (u1_0 + un * bcu * texelFetchBuffer(data, index + orderU + i    )) * (1.0 - uv[0]);                          \n\
            u1_1 = (u1_1 + un * bcu * texelFetchBuffer(data, index + orderU + i + 1)) * (1.0 - uv[0]);                          \n\
          }                                                                                                                     \n\
                                                                                                                                \n\
          u0_0 += un * uv[0] * texelFetchBuffer(data, index           + deg_u - 1);                                             \n\
          u0_1 += un * uv[0] * texelFetchBuffer(data, index           + deg_u    );                                             \n\
          u1_0 += un * uv[0] * texelFetchBuffer(data, index + orderU + deg_u - 1);                                              \n\
          u1_1 += un * uv[0] * texelFetchBuffer(data, index + orderU + deg_u    );                                              \n\
                                                                                                                                \n\
          /* point in first and second row */                                                                                   \n\
          vec4 u0 = (1.0 - uv[0]) * u0_0 + uv[0] * u0_1;                                                                        \n\
          vec4 u1 = (1.0 - uv[0]) * u1_0 + uv[0] * u1_1;                                                                        \n\
                                                                                                                                \n\
          /**************************************** 2. step : inner loop for rows 3 to order - 1 ***********************/       \n\
          float bcv = 1.0;                                                                                                      \n\
          float vn = 1.0;                                                                                                       \n\
                                                                                                                                \n\
          vec4 v0 = u0 * (1.0 - uv[1]);                                                                                         \n\
          vec4 v1 = u1 * (1.0 - uv[1]);                                                                                         \n\
                                                                                                                                \n\
          vec4 ui, ui_0, ui_1;                                                                                                  \n\
          for (i = 1; i <= deg_v - 2; ++i) {                                                                                    \n\
            bcu = 1.0;                                                                                                          \n\
            un = 1.0;                                                                                                           \n\
            ui_0 = texelFetchBuffer(data, index + (i+1) * orderU)     * (1.0 - uv[0]);                                          \n\
            ui_1 = texelFetchBuffer(data, index + (i+1) * orderU + 1) * (1.0 - uv[0]);                                          \n\
                                                                                                                                \n\
            int j;                                                                                                              \n\
            for (j = 1; j <= deg_u-2; ++j) {                                                                                    \n\
              un = un * uv[0];                                                                                                  \n\
              bcu = bcu * (float(deg_u - j) / float(j));                                                                        \n\
              ui_0 = (ui_0 + un * bcu * texelFetchBuffer(data, index + (i+1) * orderU + j    )) * (1.0 - uv[0]);                \n\
              ui_1 = (ui_1 + un * bcu * texelFetchBuffer(data, index + (i+1) * orderU + j + 1)) * (1.0 - uv[0]);                \n\
            }                                                                                                                   \n\
            ui_0 = ui_0 + un * uv[0] * texelFetchBuffer(data, index + (i+1) * orderU + deg_u - 1);                              \n\
            ui_1 = ui_1 + un * uv[0] * texelFetchBuffer(data, index + (i+1) * orderU + deg_u    );                              \n\
            ui = (1.0 - uv[0]) * ui_0 + uv[0] * ui_1;                                                                           \n\
                                                                                                                                \n\
            u0 = u1;                                                                                                            \n\
            u1 = ui;                                                                                                            \n\
                                                                                                                                \n\
            vn = vn * uv[1];                                                                                                    \n\
            bcv = bcv * (float(deg_v - i) / float(i));                                                                          \n\
            v0 = (v0 + vn * bcv * u0) * (1.0 - uv[1]);                                                                          \n\
            v1 = (v1 + vn * bcv * u1) * (1.0 - uv[1]);                                                                          \n\
          }                                                                                                                     \n\
                                                                                                                                \n\
          /**************************************** 3. step : horner scheme for last row *******************************/       \n\
          bcu = 1.0;                                                                                                            \n\
          un = 1.0;                                                                                                             \n\
          ui_0 = texelFetchBuffer(data, index + deg_v * orderU    ) * (1.0 - uv[0]);                                            \n\
          ui_1 = texelFetchBuffer(data, index + deg_v * orderU + 1) * (1.0 - uv[0]);                                            \n\
                                                                                                                                \n\
          for (i = 1; i <= deg_u-2; ++i) {                                                                                      \n\
            un = un * uv[0];                                                                                                    \n\
            bcu = bcu * (float(deg_u-i) / float(i));                                                                            \n\
            ui_0 = (ui_0 + un * bcu * texelFetchBuffer(data, index + deg_v * orderU + i    )) * (1.0 - uv[0]);                  \n\
            ui_1 = (ui_1 + un * bcu * texelFetchBuffer(data, index + deg_v * orderU + i + 1)) * (1.0 - uv[0]);                  \n\
          }                                                                                                                     \n\
                                                                                                                                \n\
          ui_0 = ui_0 + un * uv[0] * texelFetchBuffer(data, index + deg_v * orderU + deg_u - 1);                                \n\
          ui_1 = ui_1 + un * uv[0] * texelFetchBuffer(data, index + deg_v * orderU + deg_u    );                                \n\
          ui = (1.0 - uv[0]) * ui_0 + uv[0] * ui_1;                                                                             \n\
                                                                                                                                \n\
          /**************************************** 4. step : final interpolation over v ********************************/      \n\
          v0 += vn * uv[1] * u1;                                                                                                \n\
          v1 += vn * uv[1] * ui;                                                                                                \n\
                                                                                                                                \n\
          p = (1.0 - uv[1]) * v0 + uv[1] * v1;                                                                                  \n\
          p = p/p[3];                                                                                                           \n\
        }                                                                                                                       \n\
    ");
}

////////////////////////////////////////////////////////////////////////////////

std::string const
NURBSShader::horner_derivatives ()
{
    return std::string("                                                                                                        \n\
                                                                                                                                \n\
        void                                                                                                                    \n\
        evaluateSurface(in samplerBuffer  data,                                                                                 \n\
                        in int            index,                                                                                \n\
                        in int            orderU,                                                                               \n\
                        in int            orderV,                                                                               \n\
                        in vec2           uv,                                                                                   \n\
                        out vec4          p,                                                                                    \n\
                        out vec4          du,                                                                                   \n\
                        out vec4          dv)                                                                                   \n\
        {                                                                                                                       \n\
          p  = vec4(0.0);                                                                                                       \n\
          du = vec4(0.0);                                                                                                       \n\
          dv = vec4(0.0);                                                                                                       \n\
                                                                                                                                \n\
          float bcu = 1.0;                                                                                                      \n\
          float un  = 1.0;                                                                                                      \n\
                                                                                                                                \n\
          int deg_u = orderU - 1;                                                                                               \n\
          int deg_v = orderV - 1;                                                                                               \n\
                                                                                                                                \n\
          vec4 u0_0 = texelFetch(data, index              ) * (1.0 - uv[0]);                                                    \n\
          vec4 u0_1 = texelFetch(data, index + 1          ) * (1.0 - uv[0]);                                                    \n\
          vec4 u1_0 = texelFetch(data, index + orderU    )  * (1.0 - uv[0]);                                                    \n\
          vec4 u1_1 = texelFetch(data, index + orderU + 1)  * (1.0 - uv[0]);                                                    \n\
                                                                                                                                \n\
          /**************************************** 1. step : horner for first 2 rows *********************************/        \n\
          int i;                                                                                                                \n\
          for (i = 1; i <= deg_u - 2; ++i) {                                                                                    \n\
            un = un * uv[0];                                                                                                    \n\
            bcu = bcu * (float(deg_u - i) / float(i));                                                                          \n\
                                                                                                                                \n\
            u0_0 = (u0_0 + un * bcu * texelFetch(data, index + i    ))           * (1.0 - uv[0]);                               \n\
            u0_1 = (u0_1 + un * bcu * texelFetch(data, index + i + 1))           * (1.0 - uv[0]);                               \n\
                                                                                                                                \n\
            u1_0 = (u1_0 + un * bcu * texelFetch(data, index + orderU + i    )) * (1.0 - uv[0]);                                \n\
            u1_1 = (u1_1 + un * bcu * texelFetch(data, index + orderU + i + 1)) * (1.0 - uv[0]);                                \n\
          }                                                                                                                     \n\
                                                                                                                                \n\
          u0_0 += un * uv[0] * texelFetch(data, index          + deg_u - 1);                                                    \n\
          u0_1 += un * uv[0] * texelFetch(data, index          + deg_u    );                                                    \n\
          u1_0 += un * uv[0] * texelFetch(data, index + orderU + deg_u - 1);                                                    \n\
          u1_1 += un * uv[0] * texelFetch(data, index + orderU + deg_u    );                                                    \n\
                                                                                                                                \n\
          /* point in first and second row */                                                                                   \n\
          vec4 u0 = (1.0 - uv[0]) * u0_0 + uv[0] * u0_1;                                                                        \n\
          vec4 u1 = (1.0 - uv[0]) * u1_0 + uv[0] * u1_1;                                                                        \n\
                                                                                                                                \n\
          /**************************************** 2. step : inner loop for rows 3 to order - 1 ***********************/       \n\
          float bcv = 1.0;                                                                                                      \n\
          float vn = 1.0;                                                                                                       \n\
                                                                                                                                \n\
          vec4 v0 = u0 * (1.0 - uv[1]);                                                                                         \n\
          vec4 v1 = u1 * (1.0 - uv[1]);                                                                                         \n\
                                                                                                                                \n\
          vec4 ui, ui_0, ui_1;                                                                                                  \n\
          for (i = 1; i <= deg_v - 2; ++i)                                                                                      \n\
          {                                                                                                                     \n\
            bcu = 1.0;                                                                                                          \n\
            un = 1.0;                                                                                                           \n\
            ui_0 = texelFetch(data, index + (i+1) * orderU)     * (1.0 - uv[0]);                                                \n\
            ui_1 = texelFetch(data, index + (i+1) * orderU + 1) * (1.0 - uv[0]);                                                \n\
                                                                                                                                \n\
            int j;                                                                                                              \n\
            for (j = 1; j <= deg_u-2; ++j)                                                                                      \n\
            {                                                                                                                   \n\
              un = un * uv[0];                                                                                                  \n\
              bcu = bcu * (float(deg_u - j) / float(j));                                                                        \n\
              ui_0 = (ui_0 + un * bcu * texelFetch(data, index + (i+1) * orderU + j    )) * (1.0 - uv[0]);                      \n\
              ui_1 = (ui_1 + un * bcu * texelFetch(data, index + (i+1) * orderU + j + 1)) * (1.0 - uv[0]);                      \n\
            }                                                                                                                   \n\
            ui_0 = ui_0 + un * uv[0] * texelFetch(data, index + (i+1) * orderU + deg_u - 1);                                    \n\
            ui_1 = ui_1 + un * uv[0] * texelFetch(data, index + (i+1) * orderU + deg_u    );                                    \n\
            ui = (1.0 - uv[0]) * ui_0 + uv[0] * ui_1;                                                                           \n\
                                                                                                                                \n\
            u0 = u1;                                                                                                            \n\
            u1 = ui;                                                                                                            \n\
                                                                                                                                \n\
            vn = vn * uv[1];                                                                                                    \n\
            bcv = bcv * (float(deg_v - i) / float(i));                                                                          \n\
            v0 = (v0 + vn * bcv * u0) * (1.0 - uv[1]);                                                                          \n\
            v1 = (v1 + vn * bcv * u1) * (1.0 - uv[1]);                                                                          \n\
          }                                                                                                                     \n\
                                                                                                                                \n\
          /**************************************** 3. step : horner scheme for last row *******************************/       \n\
          bcu = 1.0;                                                                                                            \n\
          un = 1.0;                                                                                                             \n\
          ui_0 = texelFetch(data, index + deg_v * orderU    ) * (1.0 - uv[0]);                                                  \n\
          ui_1 = texelFetch(data, index + deg_v * orderU + 1) * (1.0 - uv[0]);                                                  \n\
                                                                                                                                \n\
          for (i = 1; i <= deg_u-2; ++i)                                                                                        \n\
          {                                                                                                                     \n\
            un = un * uv[0];                                                                                                    \n\
            bcu = bcu * (float(deg_u-i) / float(i));                                                                            \n\
            ui_0 = (ui_0 + un * bcu * texelFetch(data, index + deg_v * orderU + i    )) * (1.0 - uv[0]);                        \n\
            ui_1 = (ui_1 + un * bcu * texelFetch(data, index + deg_v * orderU + i + 1)) * (1.0 - uv[0]);                        \n\
          }                                                                                                                     \n\
                                                                                                                                \n\
          ui_0 = ui_0 + un * uv[0] * texelFetch(data, index + deg_v * orderU + deg_u - 1);                                      \n\
          ui_1 = ui_1 + un * uv[0] * texelFetch(data, index + deg_v * orderU + deg_u    );                                      \n\
          ui = (1.0 - uv[0]) * ui_0 + uv[0] * ui_1;                                                                             \n\
                                                                                                                                \n\
          /**************************************** 4. step : final interpolation over v ********************************/      \n\
          v0 += vn * uv[1] * u1;                                                                                                \n\
          v1 += vn * uv[1] * ui;                                                                                                \n\
                                                                                                                                \n\
          p = (1.0 - uv[1]) * v0 + uv[1] * v1;                                                                                  \n\
                                                                                                                                \n\
          /* transform to euclidian space */                                                                                    \n\
          dv = (orderV - 1) * ((v0[3] * v1[3]) / (p[3] * p[3])) * ((v1 / v1[3]) - (v0 / v0[3]));                                \n\
                                                                                                                                \n\
          /************************************ 5.step : dartial derivative over u ***********************************/         \n\
          bcv = 1.0;                                                                                                            \n\
          vn = 1.0;                                                                                                             \n\
                                                                                                                                \n\
          vec4 v0_0 = texelFetch(data, index              ) * (1.0 - uv[1]);                                                    \n\
          vec4 v0_1 = texelFetch(data, index + orderU    ) * (1.0 - uv[1]);                                                     \n\
          vec4 v1_0 = texelFetch(data, index + 1          ) * (1.0 - uv[1]);                                                    \n\
          vec4 v1_1 = texelFetch(data, index + orderU + 1) * (1.0 - uv[1]);                                                     \n\
          ") + std::string("                                                                                                    \n\
          for (i = 1; i <= deg_v - 2; ++i)                                                                                      \n\
          {                                                                                                                     \n\
            vn = vn * uv[1];                                                                                                    \n\
            bcv = bcv * (float(deg_v - i) / float(i));                                                                          \n\
                                                                                                                                \n\
            v0_0 = (v0_0 + vn * bcv * texelFetch(data, index + (i  ) * orderU    )) * (1.0 - uv[1]);                            \n\
            v0_1 = (v0_1 + vn * bcv * texelFetch(data, index + (i+1) * orderU    )) * (1.0 - uv[1]);                            \n\
                                                                                                                                \n\
            v1_0 = (v1_0 + vn * bcv * texelFetch(data, index + (i  ) * orderU + 1)) * (1.0 - uv[1]);                            \n\
            v1_1 = (v1_1 + vn * bcv * texelFetch(data, index + (i+1) * orderU + 1)) * (1.0 - uv[1]);                            \n\
          }                                                                                                                     \n\
                                                                                                                                \n\
          v0_0 = v0_0 + vn * uv[1] * texelFetch(data, index + (deg_v-1) * orderU    );                                          \n\
          v0_1 = v0_1 + vn * uv[1] * texelFetch(data, index + (deg_v  ) * orderU    );                                          \n\
          v1_0 = v1_0 + vn * uv[1] * texelFetch(data, index + (deg_v-1) * orderU + 1);                                          \n\
          v1_1 = v1_1 + vn * uv[1] * texelFetch(data, index + (deg_v  ) * orderU + 1);                                          \n\
                                                                                                                                \n\
          /* point in first and second row */                                                                                   \n\
          v0 = (1.0 - uv[1]) * v0_0 + uv[1] * v0_1;                                                                             \n\
          v1 = (1.0 - uv[1]) * v1_0 + uv[1] * v1_1;                                                                             \n\
                                                                                                                                \n\
          /*********************************** 6. step : for all columns *******************************************/           \n\
          bcu = 1.0;                                                                                                            \n\
          un = 1.0;                                                                                                             \n\
                                                                                                                                \n\
          u0 = v0 * (1.0 - uv[0]);                                                                                              \n\
          u1 = v1 * (1.0 - uv[0]);                                                                                              \n\
                                                                                                                                \n\
          vec4 vi_0, vi_1, vi;                                                                                                  \n\
          for (i = 1; i <= deg_u - 2; ++i)                                                                                      \n\
          {                                                                                                                     \n\
            bcv = 1.0;                                                                                                          \n\
            vn = 1.0;                                                                                                           \n\
            vi_0 = texelFetch(data, index +           i + 1) * (1.0 - uv[1]);                                                   \n\
            vi_1 = texelFetch(data, index + orderU + i + 1) * (1.0 - uv[1]);                                                    \n\
                                                                                                                                \n\
            int j;                                                                                                              \n\
            for (j = 1; j <= deg_v-2; ++j)                                                                                      \n\
            {                                                                                                                   \n\
              vn = vn * uv[1];                                                                                                  \n\
              bcv = bcv * (float(deg_v - j) / float(j));                                                                        \n\
              vi_0 = (vi_0 + vn * bcv * texelFetch(data, index + (j  ) * orderU + i + 1)) * (1.0 - uv[1]);                      \n\
              vi_1 = (vi_1 + vn * bcv * texelFetch(data, index + (j+1) * orderU + i + 1)) * (1.0 - uv[1]);                      \n\
            }                                                                                                                   \n\
            vi_0 = vi_0 + vn * uv[1] * texelFetch(data, index + (deg_v-1) * orderU + i + 1);                                    \n\
            vi_1 = vi_1 + vn * uv[1] * texelFetch(data, index + (deg_v  ) * orderU + i + 1);                                    \n\
            vi = (1.0 - uv[1]) * vi_0 + uv[1] * vi_1;                                                                           \n\
                                                                                                                                \n\
            v0 = v1;                                                                                                            \n\
            v1 = vi;                                                                                                            \n\
                                                                                                                                \n\
            un = un * uv[0];                                                                                                    \n\
            bcu = bcu * (float(deg_u - i) / float(i));                                                                          \n\
            u0 = (u0 + un * bcu * v0) * (1.0 - uv[0]);                                                                          \n\
            u1 = (u1 + un * bcu * v1) * (1.0 - uv[0]);                                                                          \n\
          }                                                                                                                     \n\
                                                                                                                                \n\
          /********************************* 7. horner step for last column ****************************************/           \n\
          bcv = 1.0;                                                                                                            \n\
          vn = 1.0;                                                                                                             \n\
          vi_0 = texelFetch(data, index +           deg_u      ) * (1.0 - uv[1]);                                               \n\
          vi_1 = texelFetch(data, index + orderU + deg_u      ) * (1.0 - uv[1]);                                                \n\
                                                                                                                                \n\
          for (i = 1; i <= deg_v-2; ++i)                                                                                        \n\
          {                                                                                                                     \n\
            vn = vn * uv[1];                                                                                                    \n\
            bcv = bcv * (float(deg_v-i) / float(i));                                                                            \n\
            vi_0 = (vi_0 + vn * bcv * texelFetch(data, index + (i  ) * orderU + deg_u)) * (1.0 - uv[1]);                        \n\
            vi_1 = (vi_1 + vn * bcv * texelFetch(data, index + (i+1) * orderU + deg_u)) * (1.0 - uv[1]);                        \n\
           }                                                                                                                    \n\
                                                                                                                                \n\
          vi_0 = vi_0 + vn * uv[1] * texelFetch(data, index + (deg_v-1) * orderU + deg_u);                                      \n\
          vi_1 = vi_1 + vn * uv[1] * texelFetch(data, index + (deg_v  ) * orderU + deg_u);                                      \n\
          vi = (1.0 - uv[1]) * vi_0 + uv[1] * vi_1;                                                                             \n\
                                                                                                                                \n\
          /******************************* 8. final interpolation ***************************************************/          \n\
          u0 += un * uv[0] * v1;                                                                                                \n\
          u1 += un * uv[0] * vi;                                                                                                \n\
                                                                                                                                \n\
          /* transform to euclidian space */                                                                                    \n\
          du = deg_u * ((u0[3] * u1[3]) / (p[3] * p[3])) * ((u1 / u1[3]) - (u0 / u0[3]));                                       \n\
          p = p/p[3];                                                                                                           \n\
        }                                                                                                                       \n\
    ");
}

////////////////////////////////////////////////////////////////////////////////
std::string const
NURBSShader::curve_horner_evaluation ()
{
    return std::string("                                                                                                       \n\
         /*******************************************************************************                                      \n\
         * Evaluate Curve using modificated horner algorithm in Bernstein basis        *                                       \n\
         *   - points are supposed to be in hyperspace : [wx, wy, w]                   *                                       \n\
         *   - curvedata[index] is the first point of curve                            *                                       \n\
         *   - t is the parameter the curve is to be evaluated for                     *                                       \n\
         ******************************************************************************/                                       \n\
        void                                                                                                                   \n\
        evaluateCurve ( in samplerBuffer data,                                                                                 \n\
                        in int index,                                                                                          \n\
                        in int order,                                                                                          \n\
                        in float t,                                                                                            \n\
                        out vec4 p )                                                                                           \n\
        {                                                                                                                      \n\
          int deg = order - 1;                                                                                                 \n\
          float u = 1.0 - t;                                                                                                   \n\
                                                                                                                               \n\
          float bc = 1.0;                                                                                                      \n\
          float tn = 1.0;                                                                                                      \n\
          p  = texelFetch(data, index);                                                                                        \n\
          p *= u;                                                                                                              \n\
                                                                                                                               \n\
          if (order > 2) {                                                                                                     \n\
            for (int i = 1; i <= deg - 1; ++i) {                                                                               \n\
              tn *= t;                                                                                                         \n\
              bc *= (float(deg-i+1) / float(i));                                                                               \n\
              p = (p + tn * bc * texelFetch(data, index + i)) * u;                                                             \n\
            }                                                                                                                  \n\
            p += tn * t * texelFetch(data, index + deg);                                                                       \n\
          } else {                                                                                                             \n\
            /* linear piece*/                                                                                                  \n\
            p = mix(texelFetch(data, index), texelFetch(data, index + 1), t);                                                  \n\
          }                                                                                                                    \n\
                                                                                                                               \n\
          /* project into euclidian coordinates */                                                                             \n\
          p[0] = p[0]/p[2];                                                                                                    \n\
          p[1] = p[1]/p[2];                                                                                                    \n\
        }                                                                                                                      \n\
    ");
}

////////////////////////////////////////////////////////////////////////////////
std::string const
NURBSShader::control_polygon_length    ()
{
    return std::string("                                                                                                                                     \n\
        vec4 control_polygon_length(in samplerBuffer	data,                                                                                                  \n\
                                    in mat4           mvp_matrix,                                                                                            \n\
                                    in int		        offset,                                                                                                \n\
                                    in int 		        u,                                                                                                     \n\
                                    in int		        v,                                                                                                     \n\
                                    in int            screen_res_x,                                                                                          \n\
                                    in int            screen_res_y )                                                                                         \n\
        {                                                                                                                                                    \n\
            int i, j;                                                                                                                                        \n\
            vec4 output = vec4(0.0);                                                                                                                         \n\
                                                                                                                                                             \n\
            //      3                                                                                                                                        \n\
            //	3------2                                                                                                                                     \n\
            //	|      |                                                                                                                                     \n\
            //0 |      | 2                                                                                                                                   \n\
            //  |      |                                                                                                                                     \n\
            //  0------1                                                                                                                                     \n\
            //      1                                                                                                                                        \n\
                                                                                                                                                             \n\
            /*For Edge 03*/                                                                                                                                  \n\
            for ( i = u; i < u * v - u + 1; i += u ) {                                                                                                       \n\
                output[0] += edge_length(texelFetch(data, offset + i).xyz, texelFetch(data, offset + i - u).xyz, mvp_matrix, screen_res_x, screen_res_y);    \n\
            }                                                                                                                                                \n\
                                                                                                                                                             \n\
            /*For Edge 01*/                                                                                                                                  \n\
            for ( i = 1; i < u; ++i ) {                                                                                                                      \n\
                output[1] += edge_length(texelFetch(data, offset + i).xyz, texelFetch(data, offset + i - 1).xyz, mvp_matrix, screen_res_x, screen_res_y);    \n\
            }                                                                                                                                                \n\
                                                                                                                                                             \n\
            /*For Edge 12*/                                                                                                                                  \n\
            for ( i = 2 * u - 1; i < u * v; i += u ) {                                                                                                       \n\
                output[2] += edge_length(texelFetch(data, offset + i).xyz, texelFetch(data, offset + i - u).xyz, mvp_matrix, screen_res_x, screen_res_y);    \n\
            }                                                                                                                                                \n\
                                                                                                                                                             \n\
            /*For Edge 23*/                                                                                                                                  \n\
            for ( i = u * v - u + 1; i < u * v; ++i ) {                                                                                                      \n\
                output[3] += edge_length(texelFetch(data, offset + i).xyz, texelFetch(data, offset + i - 1).xyz, mvp_matrix, screen_res_x, screen_res_y);    \n\
            }                                                                                                                                                \n\
                                                                                                                                                             \n\
            return output;                                                                                                                                   \n\
        }                                                                                                                                                    \n\
    ");
}

////////////////////////////////////////////////////////////////////////////////
std::string const
NURBSShader::edge_length ()
{
    return std::string("                                                                          \n\
        float edge_length(in vec3 v1,                                                             \n\
                          in vec3 v2,                                                             \n\
                          in mat4 mvp_matrix,                                                     \n\
                          in int screen_resolution_x,                                             \n\
                          in int screen_resolution_y )                                            \n\
        {                                                                                         \n\
          vec4 cs_v1 = mvp_matrix * vec4(v1, 1.0);                                                \n\
          vec4 cs_v2 = mvp_matrix * vec4(v2, 1.0);                                                \n\
                                                                                                  \n\
          cs_v1 = cs_v1 / cs_v1.w;                                                                \n\
          cs_v2 = cs_v2 / cs_v2.w;                                                                \n\
                                                                                                  \n\
          vec2 dv1 = (cs_v1.xy * 0.5 + 0.5) * vec2(screen_resolution_x, screen_resolution_y);     \n\
          vec2 dv2 = (cs_v2.xy * 0.5 + 0.5) * vec2(screen_resolution_x, screen_resolution_y);     \n\
                                                                                                  \n\
          vec2 edge = dv1.xy - dv2.xy;                                                            \n\
                                                                                                  \n\
          return clamp(length(edge), 0, max(screen_resolution_x, screen_resolution_y));           \n\
        }                                                                                         \n\
    ");
}


////////////////////////////////////////////////////////////////////////////////
std::string const
NURBSShader::edge_tess_level ()
{
    return std::string("                                                                         \n\
        float edge_tesslevel(in float length,                                                    \n\
                             in float max_error )                                                \n\
        {                                                                                        \n\
          return clamp ( length / max_error, 1.0f, 64.0f );                                      \n\
        }                                                                                        \n\
    ");
}


////////////////////////////////////////////////////////////////////////////////
std::string const
NURBSShader::inner_tess_level ()
{
    return std::string("                                                                                                      \n\
        float inner_tess_level(in samplerBuffer data,                                                                         \n\
                               in int offset,                                                                                 \n\
                               in mat4 mvp_matrix,                                                                            \n\
                               in float max_error,                                                                            \n\
                               in int screen_res_x,                                                                           \n\
                               in int screen_res_y )                                                                          \n\
        {                                                                                                                     \n\
          vec4 bbox_min = texelFetch(data, offset + 2);                                                                       \n\
          vec4 bbox_max = texelFetch(data, offset + 3);                                                                       \n\
          vec3 combin[8];                                                                                                     \n\
          int i, j;                                                                                                           \n\
          float max_length = 0.0;                                                                                             \n\
                                                                                                                              \n\
          combin[0] = to_screen_space(vec3(bbox_min.x, bbox_min.y, bbox_min.z), mvp_matrix, screen_res_x, screen_res_y).xyz;  \n\
          combin[1] = to_screen_space(vec3(bbox_min.x, bbox_min.y, bbox_max.z), mvp_matrix, screen_res_x, screen_res_y).xyz;  \n\
          combin[2] = to_screen_space(vec3(bbox_min.x, bbox_max.y, bbox_min.z), mvp_matrix, screen_res_x, screen_res_y).xyz;  \n\
          combin[3] = to_screen_space(vec3(bbox_min.x, bbox_max.y, bbox_max.z), mvp_matrix, screen_res_x, screen_res_y).xyz;  \n\
          combin[4] = to_screen_space(vec3(bbox_max.x, bbox_min.y, bbox_min.z), mvp_matrix, screen_res_x, screen_res_y).xyz;  \n\
          combin[5] = to_screen_space(vec3(bbox_max.x, bbox_min.y, bbox_max.z), mvp_matrix, screen_res_x, screen_res_y).xyz;  \n\
          combin[6] = to_screen_space(vec3(bbox_max.x, bbox_max.y, bbox_min.z), mvp_matrix, screen_res_x, screen_res_y).xyz;  \n\
          combin[7] = to_screen_space(vec3(bbox_max.x, bbox_max.y, bbox_max.z), mvp_matrix, screen_res_x, screen_res_y).xyz;  \n\
                                                                                                                              \n\
          for ( i = 0; i < 7; i++ )                                                                                           \n\
          {                                                                                                                   \n\
            for ( j = i + 1; j < 8; j++ )                                                                                     \n\
            {                                                                                                                 \n\
              float temp_length = length(combin[i].xy - combin[j].xy);                                                        \n\
                                                                                                                              \n\
              if ( temp_length > max_length )                                                                                 \n\
              {                                                                                                               \n\
                max_length = temp_length;                                                                                     \n\
              }                                                                                                               \n\
            }                                                                                                                 \n\
          }                                                                                                                   \n\
                                                                                                                              \n\
          max_length = clamp(max_length, 0, max(screen_res_x, screen_res_y) );                                                \n\
                                                                                                                              \n\
          return clamp(ceil(max_length / max_error), 2.0, 64.0);                                                              \n\
        }                                                                                                                     \n\
    ");
}


////////////////////////////////////////////////////////////////////////////////
std::string const
NURBSShader::is_inside ()
{
    return std::string("                                                                                                            \n\
        bool is_inside(vec4 point)                                                                                                  \n\
        {                                                                                                                           \n\
          if( (point.x >= -1.0 && point.x <= 1.0) && (point.y >= -1.0 && point.y <= 1.0) && (point.z >= -1.0 && point.z <= 1.0) ) { \n\
            return true;                                                                                                            \n\
          }                                                                                                                         \n\
                                                                                                                                    \n\
          return false;                                                                                                             \n\
        }                                                                                                                           \n\
    ");
}


////////////////////////////////////////////////////////////////////////////////
std::string const
NURBSShader::to_screen_space ()
{
    return std::string("                                                                                \n\
        vec4 to_screen_space(in vec3 point,                                                             \n\
                             in mat4 mvp_matrix,                                                        \n\
                             in int screen_res_x,                                                       \n\
                             in int screen_res_y )                                                      \n\
        {                                                                                               \n\
          vec4 ret_point;                                                                               \n\
                                                                                                        \n\
          ret_point = mvp_matrix * vec4(point, 1.0);                                                    \n\
          ret_point = ret_point / ret_point.w;                                                          \n\
          ret_point = vec4((ret_point.xy * 0.5 + 0.5) * vec2(screen_res_x, screen_res_y), 1.0, 1.0);    \n\
                                                                                                        \n\
          return ret_point;                                                                             \n\
        }                                                                                               \n\
    ");
}


////////////////////////////////////////////////////////////////////////////////
std::string const
NURBSShader::frustum_cull ()
{
    return std::string("                                                                                \n\
        bool frustum_cull ( in mat4 mvp_matrix )                                                        \n\
        {                                                                                               \n\
          //Frustum Culling in Clip Space                                                               \n\
                                                                                                        \n\
          vec4 cs_v1 = mvp_matrix * vec4(vPosition[0], 1.0);                                            \n\
          vec4 cs_v2 = mvp_matrix * vec4(vPosition[1], 1.0);                                            \n\
          vec4 cs_v3 = mvp_matrix * vec4(vPosition[2], 1.0);                                            \n\
          vec4 cs_v4 = mvp_matrix * vec4(vPosition[3], 1.0);                                            \n\
                                                                                                        \n\
          cs_v1 = cs_v1 / cs_v1.w;                                                                      \n\
          cs_v2 = cs_v2 / cs_v2.w;                                                                      \n\
          cs_v3 = cs_v3 / cs_v3.w;                                                                      \n\
          cs_v4 = cs_v4 / cs_v4.w;                                                                      \n\
                                                                                                        \n\
          if ( is_inside(cs_v1) || is_inside(cs_v2) || is_inside(cs_v3) || is_inside(cs_v4) )           \n\
          {                                                                                             \n\
            return true;                                                                                \n\
          }                                                                                             \n\
                                                                                                        \n\
          return false;                                                                                 \n\
        }                                                                                               \n\
    ");
}

////////////////////////////////////////////////////////////////////////////////
std::string const
NURBSShader::binary_search ()
{
    return std::string("                                      \n\
                                                              \n\
        bool                                                  \n\
        binary_search ( in samplerBuffer buf,                 \n\
                        in float         value,               \n\
                        in int           id,                  \n\
                        in int           intervals,           \n\
                        inout vec4       result )             \n\
        {                                                     \n\
          result = vec4(0.0);                                 \n\
                                                              \n\
          int id_min = id;                                    \n\
          int id_max = id + intervals - 1;                    \n\
                                                              \n\
          vec4 tmp = vec4(0.0);                               \n\
          bool found = false;                                 \n\
                                                              \n\
          while ( id_min <= id_max )                          \n\
          {                                                   \n\
            int id = id_min + (id_max - id_min) / int(2);     \n\
                                                              \n\
            tmp = texelFetch(buf, id);                        \n\
                                                              \n\
            if (value >= tmp[0] && value <= tmp[1])           \n\
            {                                                 \n\
              result = tmp;                                   \n\
              found    = true;                                \n\
              break;                                          \n\
            } else {                                          \n\
              if ( value < tmp[0] )                           \n\
              {                                               \n\
                id_max = id - 1;                              \n\
              } else {                                        \n\
                id_min = id + 1;                              \n\
              }                                               \n\
            }                                                 \n\
          }                                                   \n\
                                                              \n\
          if (found)                                          \n\
          {                                                   \n\
            return found;                                     \n\
          } else {                                            \n\
            result = vec4(0.0);                               \n\
            return found;                                     \n\
          }                                                   \n\
        }                                                     \n\
    ");
}

////////////////////////////////////////////////////////////////////////////////
std::string const
NURBSShader::trimming_helper_methods ()
{
    return std::string("                                      \n\
                                                              \n\
        uvec4 intToUInt4 ( uint input )                       \n\
        {                                                     \n\
          uvec4 result;                                       \n\
          result.w = (input & 0xFF000000) >> 24U;             \n\
          result.z = (input & 0x00FF0000) >> 16U;             \n\
          result.y = (input & 0x0000FF00) >> 8U;              \n\
          result.x = (input & 0x000000FF);                    \n\
          return result;                                      \n\
        }                                                     \n\
                                                              \n\
        void intToUint8_24 ( in  uint input,                  \n\
                             out uint a,                      \n\
                             out uint b )                     \n\
        {                                                     \n\
          b = (input & 0xFFFFFF00) >> 8U;                     \n\
          a = (input & 0x000000FF);                           \n\
        }                                                     \n\
    ");
}

////////////////////////////////////////////////////////////////////////////////
std::string const
NURBSShader::bisect_contour()
{
    return std::string("                                                 \n\
                                                                         \n\
        bool                                                             \n\
        bisect_contour ( in samplerBuffer buffer,                        \n\
                         in vec2          uv,                            \n\
                         in int           id,                            \n\
                         in int           intervals,                     \n\
                         in bool          uincreasing,                   \n\
                         inout int        intersections,                 \n\
                         out uint         index,                         \n\
                         out uint         order )                        \n\
        {                                                                \n\
          int id_min = id;                                               \n\
          int id_max = id + intervals - 1;                               \n\
                                                                         \n\
          index = 0;                                                     \n\
          order = 0;                                                     \n\
          bool preclassified = false;                                    \n\
                                                                         \n\
          //for ( int k = 0; k != intervals; ++k )                       \n\
          while ( id_min <= id_max && !preclassified )                   \n\
          {                                                              \n\
            int id    = id_min + (id_max - id_min) / int(2);             \n\
            vec4 p    = texelFetch(buffer, id);                          \n\
                                                                         \n\
            vec2  u   = unpackHalf2x16 ( floatBitsToUint ( p.z ) );      \n\
            uvec4 tmp = intToUInt4 ( floatBitsToUint ( p.w ) );          \n\
                                                                         \n\
            // point inside                                              \n\
            if ( uv[1] >= p[0] &&                                        \n\
                 uv[1] <= p[1] &&                                        \n\
                 uv[0] >= u[0] &&                                        \n\
                 uv[0] <= u[1] )                                         \n\
            {                                                            \n\
              intToUint8_24 ( floatBitsToUint ( p.w ), order, index );   \n\
              preclassified = true;                                      \n\
              break;                                                     \n\
            }                                                            \n\
                                                                         \n\
            // pre-classification of non-intersection                    \n\
            if ( (!uincreasing && uv[0] > u[0] && uv[1] > p[1]) ||       \n\
                 (!uincreasing && uv[0] > u[1] && uv[1] > p[0]) ||       \n\
                 ( uincreasing && uv[0] > u[0] && uv[1] < p[0]) ||       \n\
                 ( uincreasing && uv[0] > u[1] && uv[1] < p[1]) )        \n\
            {                                                            \n\
              break;                                                     \n\
            }                                                            \n\
                                                                         \n\
            // pre-classification of intersection                        \n\
            if ( (!uincreasing && uv[0] < u[0] && uv[1] < p[1]) ||       \n\
                 (!uincreasing && uv[0] < u[1] && uv[1] < p[0]) ||       \n\
                 ( uincreasing && uv[0] < u[0] && uv[1] > p[0]) ||       \n\
                 ( uincreasing && uv[0] < u[1] && uv[1] > p[1]) )        \n\
            {                                                            \n\
              ++intersections;                                           \n\
              break;                                                     \n\
            }                                                            \n\
                                                                         \n\
            // next step in binary search                                \n\
            if (uv[1] < p[1])                                            \n\
            {                                                            \n\
              id_max = id - 1;                                           \n\
            } else {                                                     \n\
              id_min = id + 1;                                           \n\
            }                                                            \n\
          }                                                              \n\
                                                                         \n\
          return preclassified;                                          \n\
        }                                                                \n\
    ");
}


////////////////////////////////////////////////////////////////////////////////
std::string const
NURBSShader::bisect_curve()
{
    return std::string("                                                           \n\
                                                                                   \n\
        void                                                                       \n\
        bisect_curve ( in samplerBuffer curvedata_buffer,                          \n\
                       in vec2          uv,                                        \n\
                       in int           index,                                     \n\
                       in int           order,                                     \n\
                       in bool          horizontally_increasing,                   \n\
                       in float         tmin,                                      \n\
                       in float         tmax,                                      \n\
                       inout int        intersections,                             \n\
                       inout int        iterations,                                \n\
                       in float         tolerance,                                 \n\
                       in int           max_iterations )                           \n\
        {                                                                          \n\
          float t = 0.0;                                                           \n\
          vec4 p  = vec4(0.0);                                                     \n\
                                                                                   \n\
          int iters = 0;                                                           \n\
                                                                                   \n\
          for (int i = 0; i < max_iterations; ++i)                                 \n\
          {                                                                        \n\
            ++iterations;                                                          \n\
            t = (tmax + tmin) / 2.0;                                               \n\
            evaluateCurve ( curvedata_buffer, index, order, t, p);                 \n\
                                                                                   \n\
            //if ( abs ( uv[1] - p[1] ) < tolerance )                              \n\
            if ( length ( uv - p.xy ) < tolerance )                                \n\
            {                                                                      \n\
              break;                                                               \n\
            }                                                                      \n\
                                                                                   \n\
            if (uv[1] > p[1]) {                                                    \n\
              tmin = t;                                                            \n\
            } else {                                                               \n\
              tmax = t;                                                            \n\
            }                                                                      \n\
                                                                                   \n\
            if ( (!horizontally_increasing && uv[0] > p[0] && uv[1] > p[1] )||     \n\
                 ( horizontally_increasing && uv[0] > p[0] && uv[1] < p[1] ) )     \n\
            {                                                                      \n\
              break;                                                               \n\
            }                                                                      \n\
                                                                                   \n\
            if ( (!horizontally_increasing && uv[0] < p[0] && uv[1] < p[1]) ||     \n\
                 (horizontally_increasing && uv[0] < p[0] && uv[1] > p[1]) )       \n\
            {                                                                      \n\
              ++intersections;                                                     \n\
              break;                                                               \n\
            }                                                                      \n\
          }                                                                        \n\
        }                                                                          \n\
    ");
}


////////////////////////////////////////////////////////////////////////////////
std::string const
NURBSShader::contour_binary_search()
{
    return std::string("                                                           \n\
                                                                                   \n\
        bool                                                                       \n\
        contour_binary_search ( in samplerBuffer buffer,                           \n\
                                in vec2          uv,                               \n\
                                in int           id,                               \n\
                                in int           intervals,                        \n\
                                in bool          uincreasing,                      \n\
                                inout int        intersections,                    \n\
                                inout int        curveindex )                      \n\
        {                                                                          \n\
          int id_min = id;                                                         \n\
          int id_max = id + intervals - 1;                                         \n\
                                                                                   \n\
          vec4 tmp = vec4(0.0);                                                    \n\
          bool found = false;                                                      \n\
                                                                                   \n\
          while ( id_min <= id_max )                                               \n\
          {                                                                        \n\
            int id = id_min + (id_max - id_min) / int(2);                          \n\
            tmp = texelFetch(buffer, id );                                         \n\
                                                                                   \n\
            if ( uv[1] >= tmp[0] && uv[1] <= tmp[1])                               \n\
            {                                                                      \n\
              if ( uv[0] >= tmp[2] && uv[0] <= tmp[3] )                            \n\
              {                                                                    \n\
                curveindex = id;                                                   \n\
                found = true;                                                      \n\
              } else {                                                             \n\
                if ( uv[0] < tmp[2] )                                              \n\
                {                                                                  \n\
                  ++intersections;                                                 \n\
                }                                                                  \n\
              }                                                                    \n\
              break;                                                               \n\
            } else {                                                               \n\
                                                                                   \n\
        #if 0                                                                      \n\
              if ( uv[1] < tmp[0] && uv[0] > tmp[3] && uincreasing ||              \n\
                   uv[1] > tmp[1] && uv[0] > tmp[3] && !uincreasing )              \n\
              {                                                                    \n\
                break;                                                             \n\
              }                                                                    \n\
                                                                                   \n\
              if ( uv[1] > tmp[1] && uv[0] < tmp[2] && uincreasing ||              \n\
                   uv[1] < tmp[0] && uv[0] < tmp[2] && !uincreasing )              \n\
              {                                                                    \n\
                ++intersections;                                                   \n\
                break;                                                             \n\
              }                                                                    \n\
        #else                                                                      \n\
              if ( ( uv[1] < tmp[1] && uv[0] > tmp[3] &&  uincreasing ) ||         \n\
                   ( uv[1] < tmp[0] && uv[0] > tmp[2] &&  uincreasing ) ||         \n\
                   ( uv[1] > tmp[0] && uv[0] > tmp[3] && !uincreasing ) ||         \n\
                   ( uv[1] > tmp[1] && uv[0] > tmp[2] && !uincreasing ) )          \n\
              {                                                                    \n\
                break;                                                             \n\
              }                                                                    \n\
                                                                                   \n\
              if ( ( uv[1] > tmp[0] && uv[0] < tmp[2] &&  uincreasing ) ||         \n\
                   ( uv[1] > tmp[1] && uv[0] < tmp[3] &&  uincreasing ) ||         \n\
                   ( uv[1] < tmp[1] && uv[0] < tmp[2] && !uincreasing ) ||         \n\
                   ( uv[1] < tmp[0] && uv[0] < tmp[3] && !uincreasing ))           \n\
              {                                                                    \n\
                ++intersections;                                                   \n\
                break;                                                             \n\
              }                                                                    \n\
        #endif                                                                     \n\
                                                                                   \n\
              if ( uv[1] < tmp[0] )                                                \n\
              {                                                                    \n\
                id_max = id - 1;                                                   \n\
              } else {                                                             \n\
                id_min = id + 1;                                                   \n\
              }                                                                    \n\
            }                                                                      \n\
          }                                                                        \n\
                                                                                   \n\
          return found;                                                            \n\
        }                                                                          \n\
    ");
}


////////////////////////////////////////////////////////////////////////////////
std::string const
NURBSShader::contour_based_trimming()
{
    return std::string("                                                                                   \n\
                                                                                                           \n\
        bool                                                                                               \n\
        trim ( in samplerBuffer partition,                                                                 \n\
               in samplerBuffer contourlist,                                                               \n\
               in samplerBuffer curvelist,                                                                 \n\
               in samplerBuffer curvedata,                                                                 \n\
               in samplerBuffer pointdata,                                                                 \n\
               in vec2          uv,                                                                        \n\
               in int           id,                                                                        \n\
               in int           trim_outer,                                                                \n\
               inout int        iters,                                                                     \n\
               in float         tolerance,                                                                 \n\
               in int           max_iterations )                                                           \n\
        {                                                                                                  \n\
          int total_intersections  = 0;                                                                    \n\
          int v_intervals          = int ( floatBitsToUint ( texelFetch ( partition, id ).x ) );           \n\
                                                                                                           \n\
          // if there is no partition in vertical(v) direction -> return                                   \n\
          if ( v_intervals == 0)                                                                           \n\
          {                                                                                                \n\
            return false;                                                                                  \n\
          }                                                                                                \n\
                                                                                                           \n\
          vec4 domaininfo2 = texelFetch ( partition, id+1 );                                               \n\
                                                                                                           \n\
          // classify against whole domain                                                                 \n\
          if ( uv[0] > domaininfo2[1] || uv[0] < domaininfo2[0] ||                                         \n\
               uv[1] > domaininfo2[3] || uv[1] < domaininfo2[2] )                                          \n\
          {                                                                                                \n\
            return bool(trim_outer);                                                                       \n\
          }                                                                                                \n\
                                                                                                           \n\
          vec4 vinterval = vec4(0.0, 0.0, 0.0, 0.0);                                                       \n\
          bool vinterval_found = binary_search ( partition, uv[1], id + 2, v_intervals, vinterval );       \n\
                                                                                                           \n\
          //if ( !vinterval_found ) {                                                                      \n\
          //  return bool(trim_outer);                                                                     \n\
          //}                                                                                              \n\
                                                                                                           \n\
          int celllist_id = int(floatBitsToUint(vinterval[2]));                                            \n\
          int ncells      = int(floatBitsToUint(vinterval[3]));                                            \n\
                                                                                                           \n\
          vec4 celllist_info  = texelFetch(partition, int(celllist_id) );                                  \n\
          vec4 cell           = vec4(0.0);                                                                 \n\
          bool cellfound      = binary_search   (partition, uv[0], celllist_id + 1, int(ncells), cell );   \n\
                                                                                                           \n\
          //if (!cellfound)                                                                                \n\
          //{                                                                                              \n\
          //  return bool(trim_outer);                                                                     \n\
          //}                                                                                              \n\
                                                                                                           \n\
          uvec4 type_ncontours     = intToUInt4 ( floatBitsToUint(cell[2]) );                              \n\
          total_intersections      = int(type_ncontours.y);                                                \n\
          int overlapping_contours = int(type_ncontours.z);                                                \n\
                                                                                                           \n\
          int contourlist_id       = int(floatBitsToUint(cell[3]));                                        \n\
                                                                                                           \n\
          for ( int i = 0; i < overlapping_contours; ++i )                                                 \n\
          {                                                                                                \n\
            vec2 contour    = texelFetch ( contourlist, contourlist_id + i ).xy;                           \n\
                                                                                                           \n\
            uvec4 ncurves_uincreasing = intToUInt4 ( floatBitsToUint(contour.x) );                         \n\
            bool contour_uincreasing  = ncurves_uincreasing.y > 0;                                         \n\
            int curves_in_contour     = int(ncurves_uincreasing.x);                                        \n\
            int  curvelist_id         = int(floatBitsToUint(contour.y));                                   \n\
                                                                                                           \n\
        //#define NO_EXTRA_CURVEINFO_BUFFER                                                                \n\
        #ifdef NO_EXTRA_CURVEINFO_BUFFER                                                                   \n\
            uint  curveid      = 0;                                                                        \n\
            uint  curveorder   = 0;                                                                        \n\
            bool process_curve = bisect_contour ( curvelist,                                               \n\
                                                  uv,                                                      \n\
                                                  curvelist_id,                                            \n\
                                                  curves_in_contour,                                       \n\
                                                  contour_uincreasing,                                     \n\
                                                  total_intersections,                                     \n\
                                                  curveid,                                                 \n\
                                                  curveorder );                                            \n\
            if ( process_curve )                                                                           \n\
            {                                                                                              \n\
              int iters = 0;                                                                               \n\
              bisect_curve ( pointdata, uv, int(curveid), int(curveorder), contour_uincreasing,            \n\
                             0.0f, 1.0f, total_intersections, iters, tolerance, max_iterations );          \n\
            }                                                                                              \n\
        #else                                                                                              \n\
            int curveid       = 0;                                                                         \n\
            bool process_curve = contour_binary_search ( curvelist,                                        \n\
                                                         uv,                                               \n\
                                                         curvelist_id,                                     \n\
                                                         curves_in_contour,                                \n\
                                                         contour_uincreasing,                              \n\
                                                         total_intersections,                              \n\
                                                         curveid );                                        \n\
            if ( process_curve )                                                                           \n\
            {                                                                                              \n\
              int iters = 0;                                                                               \n\
              float curveinfo = texelFetch (curvedata, curveid).x;                                         \n\
              uint pointid    = 0;                                                                         \n\
              uint curveorder = 0;                                                                         \n\
              intToUint8_24 ( floatBitsToUint ( curveinfo ), curveorder, pointid );                        \n\
              bisect_curve ( pointdata, uv, int(pointid), int(curveorder), contour_uincreasing,            \n\
                             0.0f, 1.0f, total_intersections, iters, tolerance, max_iterations );          \n\
            }                                                                                              \n\
        #endif                                                                                             \n\
          }                                                                                                \n\
                                                                                                           \n\
          return ( (mod(total_intersections, 2) == 1) != bool(trim_outer) );                               \n\
                                                                                                           \n\
        }                                                                                                  \n\
    ");
}


} // namespace gua
