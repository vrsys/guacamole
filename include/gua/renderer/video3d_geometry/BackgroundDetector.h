#ifndef BACKGROUNDDETECTOR_H
#define BACKGROUNDDETECTOR_H

#include <cmath>

inline
float minf(const float& a, const float& b){
	return a < b ? a : b;	
}


inline
float maxf(const float& a, const float& b){
	return a > b ? a : b;	
}


struct pixel{
	
	pixel()
	: r(0),
	g(0),
	b(0)
	{}
	
	pixel(unsigned char ri, unsigned char gi, unsigned char bi)
	: r(ri),
	g(gi),
	b(bi)
	{}
	
	unsigned char r;
	unsigned char g;
	unsigned char b;
};

struct pixel_s{
	
	pixel_s()
	: r(0),
	g(0),
	b(0)
	{}
#if 0	
	pixel_s(unsigned char ri, unsigned char gi, unsigned char bi)
	: r(ri * 256),
	g(gi * 256),
	b(bi * 256)
	{}
	
	pixel
	to_char(){
	  return pixel((unsigned char) (maxf(0.0f,minf(1.0f, r * 1.0f/65535)) * 255),
		       (unsigned char) (maxf(0.0f,minf(1.0f, g * 1.0f/65535)) * 255),
		       (unsigned char) (maxf(0.0f,minf(1.0f, b * 1.0f/65535)) * 255));
	}
	
	unsigned short r;
	unsigned short g;
	unsigned short b;
#endif
#if 1	
	pixel_s(unsigned char ri, unsigned char gi, unsigned char bi)
	: r(ri),
	g(gi),
	b(bi)
	{}
	
	pixel
	to_char(){
	  return pixel((unsigned char) (maxf(0.0f,minf(1.0f, r * 1.0f/255)) * 255),
		       (unsigned char) (maxf(0.0f,minf(1.0f, g * 1.0f/255)) * 255),
		       (unsigned char) (maxf(0.0f,minf(1.0f, b * 1.0f/255)) * 255));
	}
	
	unsigned short r;
	unsigned short g;
	unsigned short b;
#endif	
	
};

struct pixel_f{
	
	pixel_f()
	: r(0),
	g(0),
	b(0)
	{}
	
	pixel_f(unsigned char ri, unsigned char gi, unsigned char bi)
	: r(ri * 1.0f/255.0f),
	g(gi * 1.0f/255.0f),
	b(bi * 1.0f/255.0f)
	{}

	pixel_f(float ri, float gi, float bi)
	: r(ri),
	g(gi),
	b(bi)
	{}
	
	pixel
	to_char(){
	  return pixel( (unsigned char) (maxf(0.0f,minf(1.0f, r)) * 255),
			(unsigned char) (maxf(0.0f,minf(1.0f, g)) * 255),
			(unsigned char) (maxf(0.0f,minf(1.0f, b)) * 255));
}
	
	pixel_f to_yuv(){
		
		float Y = 0.299*r + 0.587*g + 0.114*b;
		float U = 0.492* (b-Y);
		float V = 0.877* (r-Y);
		return pixel_f(Y,U,V);
	}
	
	float r;
	float g;
	float b;
	
};

class BackgroundDetector{
	
public:
	
	BackgroundDetector(unsigned w, unsigned h);
	~BackgroundDetector();
	
	void reset();
	
	void detect(unsigned char* frame, float thresh = 0.64);
	unsigned char* getBackground();
	unsigned char* getForeground();
	

	void setBackground(unsigned index, unsigned char r, unsigned char g, unsigned char b);
	bool detectBackground(unsigned index, unsigned char r, unsigned char g, unsigned char b);


	void detectBackground(unsigned char* frame, float thresh = 0.64);


private:
	
	void RGB_to_CIELab();
	void CIELab_to_RGB();
	
	pixel_f RGB_to_CIELab(unsigned char r, unsigned char g, unsigned char b);

	unsigned m_w;
	unsigned m_h;
	
	pixel_f*   m_bg_f;
	
	pixel_f*   m_lab;
	pixel_s*   m_image;
	
	pixel* m_bg;
	pixel* m_fg;
	
	bool m_reset;
	
};

#endif // #ifndef  BACKGROUNDDETECTOR_H
