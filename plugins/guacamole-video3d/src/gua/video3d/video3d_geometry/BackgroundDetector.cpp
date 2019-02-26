#include <gua/video3d/video3d_geometry/BackgroundDetector.h>

#include <iostream>

namespace
{
#define FORC(cnt) for(c = 0; c < cnt; c++)
#define FORC3 FORC(3)

#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define LIM(x, min, max) MAX(min, MIN(x, max))
#define CLIP(x) LIM(x, 0, 65535)

void pseudoinverse(const double (*in)[3], double (*out)[3], int size)
{
    double work[3][6], num;
    int i, j, k;

    for(i = 0; i < 3; i++)
    {
        for(j = 0; j < 6; j++)
            work[i][j] = j == i + 3;
        for(j = 0; j < 3; j++)
            for(k = 0; k < size; k++)
                work[i][j] += in[k][i] * in[k][j];
    }
    for(i = 0; i < 3; i++)
    {
        num = work[i][i];
        for(j = 0; j < 6; j++)
            work[i][j] /= num;
        for(k = 0; k < 3; k++)
        {
            if(k == i)
                continue;
            num = work[k][i];
            for(j = 0; j < 6; j++)
                work[k][j] -= work[i][j] * num;
        }
    }
    for(i = 0; i < size; i++)
        for(j = 0; j < 3; j++)
            for(out[i][j] = k = 0; k < 3; k++)
                out[i][j] += work[j][k + 3] * in[i][k];
}

// To the extent possible under law, Manuel Llorens <manuelllorens@gmail.com>
// has waived all copyright and related or neighboring rights to this work.
// This code is licensed under CC0 v1.0, see license information at
// http://creativecommons.org/publicdomain/zero/1.0/

#if 0	
	float (*lab)[3];
	unsigned short (*image)[3];
#endif
double ep, ka;
double xyz_rgb_2[3][3];
float d50_white[3] = {0.964220, 1, 0.825211};
static const double rgb_xyz[3][3] = {{0.7976748465, 0.1351917082, 0.0313534088}, {0.2880402025, 0.7118741325, 0.0000856651}, {0.0000000000, 0.0000000000, 0.8252114389}}; // From Jacques Desmis

double f_cbrt(double r)
{
    r /= 65535.0;
    return r > ep ? pow(r, 1 / 3.0) : (ka * r + 16) / 116.0;
}

#if 0	
	void CIELab_Init(){
		int i;
		
		ep=216.0/24389.0;
		ka=24389.0/27.0;
		pseudoinverse(rgb_xyz,xyz_rgb_2,3);
		
		lab=(float (*)[3])calloc(width*height,sizeof *lab);  
		merror(lab,"CIELab_Init()");		
	}
	
	void CIELab_End(){
		if(lab) free(lab);
	}
#endif

float dist(const pixel_f& a, const pixel_f& b) { return std::sqrt((a.r - b.r) * (a.r - b.r) + (a.g - b.g) * (a.g - b.g) + (a.b - b.b) * (a.b - b.b)); }

pixel_f blend(const pixel_f& curr, const pixel_f& old)
{
    pixel_f res;
    const float alpha = 0.125f;
    res.r = alpha * curr.r + (1.0f - alpha) * old.r;
    res.g = alpha * curr.g + (1.0f - alpha) * old.g;
    res.b = alpha * curr.b + (1.0f - alpha) * old.b;
    return res;
}

} // namespace
/*
 unsigned m_w;
 unsigned m_h;

 pixel_f*   m_bg_f;
 pixel* m_bg;
 pixel* m_fg;

 */
BackgroundDetector::BackgroundDetector(unsigned w, unsigned h) : m_w(w), m_h(h), m_bg_f(0), m_lab(0), m_image(0), m_bg(0), m_fg(0), m_reset(false)
{
    m_bg_f = new pixel_f[m_w * m_h];
    m_lab = new pixel_f[m_w * m_h];
    m_image = new pixel_s[m_w * m_h];
    m_bg = new pixel[m_w * m_h];
    m_fg = new pixel[m_w * m_h];

    ep = 216.0 / 24389.0;
    ka = 24389.0 / 27.0;
    pseudoinverse(rgb_xyz, xyz_rgb_2, 3);
}

BackgroundDetector::~BackgroundDetector()
{
    delete[] m_bg_f;
    delete[] m_lab;
    delete[] m_image;
    delete[] m_bg;
    delete[] m_fg;
}

void BackgroundDetector::reset() { m_reset = true; }

void BackgroundDetector::detect(unsigned char* frame, float thresh)
{
    std::cerr << "detect: " << thresh << std::endl;

#if 0	
	for( unsigned y = 0; y < m_h; ++y){
		for(unsigned x = 0; x < m_w; ++x){
			const unsigned i = 3 * m_w * y + 3 * x;
			const unsigned j =     m_w * y + x;
			unsigned char r = frame[i + 0];
			unsigned char g = frame[i + 1];
			unsigned char b = frame[i + 2];
			
			pixel_f pf(r,g,b);
			pixel_f pf_yuv = pf.to_yuv();
			if(pf_yuv.r < 0.1){
				m_fg[j] = pixel(0,255,0);//p.to_char();
				m_bg[j] = pixel(0,0,0);
			}
			else{
				m_bg[j] = pixel(255,0,0);//p.to_char();
				m_fg[j] = pixel(0,0,0);				
			}
		}
	}
#endif
    // 1. convert frame to 16 bit image
    for(unsigned y = 0; y < m_h; ++y)
    {
        for(unsigned x = 0; x < m_w; ++x)
        {
            const unsigned i = 3 * m_w * y + 3 * x;
            const unsigned j = m_w * y + x;
            unsigned char r = frame[i + 0];
            unsigned char g = frame[i + 1];
            unsigned char b = frame[i + 2];

            m_image[j] = pixel_s(r, g, b);
        }
    }

    // 2. convert image to lab
    RGB_to_CIELab();

    if(m_reset)
    {
        m_reset = false;
        std::cerr << "resetting" << std::endl;
        for(unsigned y = 0; y < m_h; ++y)
        {
            for(unsigned x = 0; x < m_w; ++x)
            {
                const unsigned j = m_w * y + x;
                m_bg_f[j] = m_lab[j];
                m_bg[j] = pixel(255, 0, 0); // m_image[j].to_char();
                m_fg[j] = pixel(0, 0, 0);
            }
        }
    }
    else
    {
        for(unsigned y = 1; y < m_h - 1; ++y)
        {
            for(unsigned x = 1; x < m_w - 1; ++x)
            {
                const unsigned j = m_w * y + x;

                const unsigned nw = m_w * y + x;
                const unsigned nn = m_w * y + x;
                const unsigned ne = m_w * y + x;
                const unsigned ee = m_w * y + x;
                const unsigned se = m_w * y + x;
                const unsigned ss = m_w * y + x;
                const unsigned sw = m_w * y + x;
                const unsigned ww = m_w * y + x;

                const float w = 1.5f;

                const float d = dist(m_lab[j], m_bg_f[j]) + w * dist(m_lab[nw], m_bg_f[nw]) + w * dist(m_lab[nn], m_bg_f[nn]) + w * dist(m_lab[ne], m_bg_f[ne]) + w * dist(m_lab[ee], m_bg_f[ee]) +
                                w * dist(m_lab[se], m_bg_f[se]) + w * dist(m_lab[ss], m_bg_f[ss]) + w * dist(m_lab[sw], m_bg_f[sw]) + w * dist(m_lab[ww], m_bg_f[ww]);
                if(d < (thresh + 8 * w * thresh))
                {
                    m_bg[j] = pixel(255, 0, 0);
                    m_fg[j] = pixel(0, 0, 0);
                    // blend new bg into old bg
                    // m_bg_f[j] = blend(m_lab[j],m_bg_f[j]);
                }
                else
                {
                    m_bg[j] = pixel(0, 0, 0);
                    // m_fg[j] = pixel(0,255,0);
                    const unsigned i = 3 * m_w * y + 3 * x;
                    unsigned char r = frame[i + 0];
                    unsigned char g = frame[i + 1];
                    unsigned char b = frame[i + 2];
                    m_fg[j] = pixel(r, g, b);
                }
            }
        }
        for(unsigned y = 1; y < m_h - 1; ++y)
        {
            for(unsigned x = 1; x < m_w - 1; ++x)
            {
                const unsigned j = m_w * y + x;
                if(m_bg[j].r == 255)
                {
                    // blend new bg into old bg
                    // m_bg_f[j] = blend(m_lab[j],m_bg_f[j]);
                }
            }
        }
    }

#if 0	
	// 3. convert image back to rgb
	CIELab_to_RGB();
	
	// 4. apply image to unsigned char image
	for( unsigned y = 0; y < m_h; ++y){
		for(unsigned x = 0; x < m_w; ++x){
			const unsigned j =     m_w * y + x;
			m_fg[j] = m_image[j].to_char();
			m_bg[j] = m_fg[j];
		}
	}
#endif
}

unsigned char* BackgroundDetector::getBackground() { return (unsigned char*)m_bg; }

unsigned char* BackgroundDetector::getForeground() { return (unsigned char*)m_fg; }

void BackgroundDetector::RGB_to_CIELab()
{
    int c;
    double xyz[3];

#pragma omp parallel for private(offset, xyz, c) shared(lab)
    for(unsigned offset = 0; offset < m_w * m_h; offset++)
    {
        // Convert RGB to XYZ
        xyz[0] = xyz[1] = xyz[2] = 0;
        FORC3
        {
#if 0
			xyz[c]+=rgb_xyz[c][0]*(double)image[offset][0];
			xyz[c]+=rgb_xyz[c][1]*(double)image[offset][1];
			xyz[c]+=rgb_xyz[c][2]*(double)image[offset][2];
#endif
            xyz[c] += rgb_xyz[c][0] * (double)m_image[offset].r;
            xyz[c] += rgb_xyz[c][1] * (double)m_image[offset].g;
            xyz[c] += rgb_xyz[c][2] * (double)m_image[offset].b;
        }

        // Convert XYZ to L*a*b*
        FORC3 xyz[c] = f_cbrt(xyz[c] / d50_white[c]);
#if 0		
		lab[offset][0]=116.0*xyz[1]-16.0;		
		lab[offset][1]=500.0*(xyz[0]-xyz[1]);;
		lab[offset][2]=200.0*(xyz[1]-xyz[2]);
#endif
        m_lab[offset].r = 116.0 * xyz[1] - 16.0;
        m_lab[offset].g = 500.0 * (xyz[0] - xyz[1]);
        ;
        m_lab[offset].b = 200.0 * (xyz[1] - xyz[2]);
    }
}

void BackgroundDetector::CIELab_to_RGB()
{
    int c;
    double L;
    double xyz[3], rgb[3], f[3];

#pragma omp parallel for private(offset, xyz, c, f, L, rgb) shared(image)
    for(unsigned offset = 0; offset < m_w * m_h; offset++)
    {
        // Convert L*a*b* to XYZ

#if 0		
		L=(double)lab[offset][0];
		f[1]=(L+16.0)/116.0;	// fy
		f[0]=f[1]+(double)lab[offset][1]/500.0;	// fx
		f[2]=f[1]-(double)lab[offset][2]/200.0;	// fz
#endif

        L = (double)m_lab[offset].r;
        f[1] = (L + 16.0) / 116.0;                     // fy
        f[0] = f[1] + (double)m_lab[offset].g / 500.0; // fx
        f[2] = f[1] - (double)m_lab[offset].b / 200.0; // fz

        xyz[0] = 65535.0 * d50_white[0] * (f[0] * f[0] * f[0] > ep ? f[0] * f[0] * f[0] : (116.0 * f[0] - 16.0) / ka);
        xyz[1] = 65535.0 * d50_white[1] * (L > ka * ep ? pow(f[1], 3.0) : L / ka);
        xyz[2] = 65535.0 * d50_white[2] * (f[2] * f[2] * f[2] > ep ? f[2] * f[2] * f[2] : (116.0 * f[2] - 16.0) / ka);

        // Convert XYZ to RGB
        rgb[0] = rgb[1] = rgb[2] = 0;
        FORC3
        {
            rgb[0] += xyz[c] * xyz_rgb_2[c][0];
            rgb[1] += xyz[c] * xyz_rgb_2[c][1];
            rgb[2] += xyz[c] * xyz_rgb_2[c][2];
        }
#if 0
		FORC3 image[offset][c]=(unsigned short)CLIP(rgb[c]);
#endif

        m_image[offset].r = (unsigned short)CLIP(rgb[0]);
        m_image[offset].g = (unsigned short)CLIP(rgb[1]);
        m_image[offset].b = (unsigned short)CLIP(rgb[2]);
    }
}

void BackgroundDetector::setBackground(unsigned index, unsigned char r, unsigned char g, unsigned char b) { m_bg_f[index] = RGB_to_CIELab(r, g, b); }

bool BackgroundDetector::detectBackground(unsigned index, unsigned char r, unsigned char g, unsigned char b)
{
    pixel_f lab(RGB_to_CIELab(r, g, b));
    const float d = dist(lab, m_bg_f[index]);

    if(d < 0.64)
    {
        m_bg_f[index] = blend(lab, m_bg_f[index]);
        return true;
    }
    return false;
}

void BackgroundDetector::detectBackground(unsigned char* frame, float thresh)
{
    // 1. convert frame to 16 bit image
    for(unsigned y = 0; y < m_h; ++y)
    {
        for(unsigned x = 0; x < m_w; ++x)
        {
            const unsigned i = 3 * m_w * y + 3 * x;
            const unsigned j = m_w * y + x;
            unsigned char r = frame[i + 0];
            unsigned char g = frame[i + 1];
            unsigned char b = frame[i + 2];

            m_image[j] = pixel_s(r, g, b);
        }
    }

    // 2. convert image to lab
    RGB_to_CIELab();

    if(m_reset)
    {
        m_reset = false;
        std::cerr << "resetting" << std::endl;
        for(unsigned y = 0; y < m_h; ++y)
        {
            for(unsigned x = 0; x < m_w; ++x)
            {
                const unsigned j = m_w * y + x;
                m_bg_f[j] = m_lab[j];
                m_bg[j] = pixel(255, 0, 0); // m_image[j].to_char();
                m_fg[j] = pixel(0, 0, 0);
            }
        }
    }
    else
    {
        for(unsigned y = 1; y < m_h - 1; ++y)
        {
            for(unsigned x = 1; x < m_w - 1; ++x)
            {
                const unsigned j = m_w * y + x;

                const unsigned nw = m_w * y + x;
                const unsigned nn = m_w * y + x;
                const unsigned ne = m_w * y + x;
                const unsigned ee = m_w * y + x;
                const unsigned se = m_w * y + x;
                const unsigned ss = m_w * y + x;
                const unsigned sw = m_w * y + x;
                const unsigned ww = m_w * y + x;

                const float w = 1.5f;

                const float d = dist(m_lab[j], m_bg_f[j]) + w * dist(m_lab[nw], m_bg_f[nw]) + w * dist(m_lab[nn], m_bg_f[nn]) + w * dist(m_lab[ne], m_bg_f[ne]) + w * dist(m_lab[ee], m_bg_f[ee]) +
                                w * dist(m_lab[se], m_bg_f[se]) + w * dist(m_lab[ss], m_bg_f[ss]) + w * dist(m_lab[sw], m_bg_f[sw]) + w * dist(m_lab[ww], m_bg_f[ww]);
                if(d < (thresh + 8 * w * thresh))
                {
                    m_bg[j] = pixel(255, 0, 0);
                    m_fg[j] = pixel(0, 0, 0);
                    // blend new bg into old bg
                    // m_bg_f[j] = blend(m_lab[j],m_bg_f[j]);
                }
                else
                {
                    m_bg[j] = pixel(0, 0, 0);
                    // m_fg[j] = pixel(0,255,0);
                    const unsigned i = 3 * m_w * y + 3 * x;
                    unsigned char r = frame[i + 0];
                    unsigned char g = frame[i + 1];
                    unsigned char b = frame[i + 2];
                    m_fg[j] = pixel(r, g, b);
                }
            }
        }
        for(unsigned y = 1; y < m_h - 1; ++y)
        {
            for(unsigned x = 1; x < m_w - 1; ++x)
            {
                const unsigned j = m_w * y + x;
                if(m_bg[j].r == 255)
                {
                    // blend new bg into old bg
                    m_bg_f[j] = blend(m_lab[j], m_bg_f[j]);
                }
            }
        }
    }
}

pixel_f BackgroundDetector::RGB_to_CIELab(unsigned char r, unsigned char g, unsigned char b)
{
    int c;
    double xyz[3];

    pixel_f lab;

    xyz[0] = xyz[1] = xyz[2] = 0;
    FORC3
    {
        xyz[c] += rgb_xyz[c][0] * (double)r;
        xyz[c] += rgb_xyz[c][1] * (double)g;
        xyz[c] += rgb_xyz[c][2] * (double)b;
    }

    // Convert XYZ to L*a*b*
    FORC3 xyz[c] = f_cbrt(xyz[c] / d50_white[c]);

    lab.r = 116.0 * xyz[1] - 16.0;
    lab.g = 500.0 * (xyz[0] - xyz[1]);
    ;
    lab.b = 200.0 * (xyz[1] - xyz[2]);

    return lab;
}
