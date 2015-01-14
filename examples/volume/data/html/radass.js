(function($){

  
  var prefix = (function () {
  var styles = window.getComputedStyle(document.documentElement, ''),
    pre = (Array.prototype.slice
      .call(styles)
      .join('') 
      .match(/-(moz|webkit|ms)-/) || (styles.OLink === '' && ['', 'o'])
    )[1],
    dom = ('WebKit|Moz|MS|O').match(new RegExp('(' + pre + ')', 'i'))[1];
  return {
    dom: dom,
    lowercase: pre,
    css: '-' + pre + '-',
    js: pre[0].toUpperCase() + pre.substr(1)
  };
})();

//console.log(prefix);
  
  
	var psize = 196;
	var asize = 8;
	var pradius = parseInt(psize/2,10);
	var aradius = parseInt(asize/2,10);
	
	function legalAngle(a){ if(a < 0) a += 360; return a%360; }
	
	var $g = $("<div/>").css({ position:"relative", margin:"0 auto", display:"inline-block" });
	var $prev = $("<div/>").css({ overflow:"hidden", background:"white" }).width(psize).height(psize/2).appendTo($g);
	//$prev.clone(false).appendTo($g).css({position:"absolute",top:0,left:0,zIndex:-1}); // white bg
	$prev.css({zIndex:1});
	
	var barh = parseInt(asize * 2 / 3,10);
	var barw = parseInt(psize * 1.5,10);
	var bary = ((psize-barh)/2);
	var barx = -((barw-psize)/2);
	
	var $bar = $("<div/>").css({ position:"absolute",left:barx,top:bary, background:"purple", borderWidth:"1px",borderStyle:"solid",borderColor:"black" }).width(barw).height(barh).appendTo($g);
	var $angle = $("<div/>").css({ position:"absolute", left:"45%",top:"-50px", borderRadius:"50%", background:"purple", cursor:"move" }).width(asize).height(asize).hide().appendTo($g);
	
	var $delPoint = $("<div/>").css({
		position:"absolute",
		left:-20,top:0,
		backgroundImage:"url(data:image/gif;base64,R0lGODlhEAAQAKEBAICAgP///////////yH5BAEKAAIALAAAAAAQABAAAAIylC+gyAfejJp0uYCzDk/W30mbFooAdnIcc6buirToXCbq3bI4rb9qLdsBR5laAmSMNAoAOw==)",
		cursor:"pointer"
	}).width(16).height(16).on("click",function(e){
		//alert("ftw");
		colorpickerShowed.trigger("del");
		e.stopPropagation();
	});
	
	var $addPoint = $("<div/>").css({
		position:"absolute",
		left:0,top:0,
		backgroundImage:"url(data:image/gif;base64,R0lGODdhEAAQAIQaAOnp6UJCQubm5kdHRyYmJjY2NkVFRT09PUZGRkpKSj4+PklJSfHx8d7e3vj4+PX19eLi4tbW1u3t7dra2vv7+9PT0/39/dDQ0EFBQTMzM////////////////////////ywAAAAAEAAQAAAFYGAmjmQpWmiqqiLlvjAsOnRdFHUtPny/JL2eiEEsDgbFokjCbAYCzaYIAMBYr1iqSCDAejFcEWRMfpLJooZ6bUCs16KJfK44zOeiiH5PIOz3IhWCg4SEIheIiYqKJo0kIQA7)"
	}).width(16).height(16).hide().on("click",function(e){
		$bar.trigger("add");
		e.stopPropagation();
	});//.appendTo($bar);
	
	var lastDrag = 0;

	var colorpickerShowed = false;
	var addPointShowed = false;
	
	var useOpacity = true;
	
/*	$("body").on("click",function(){
		//console.log("click",colorpickerShowed);
		if(colorpickerShowed !== false)
			colorpickerShowed.trigger("del");
	});
*/	
	var $input = $("<input/>",{type:"hidden",value:"#ff00ff"}).css({position:"absolute",bottom:0,left:0}).appendTo($g);
	$input.minicolors({
		inline:true,
		opacity:useOpacity,
		change:function(cl,op){
			//cl = $input.minicolors('value');
			//op = $input.minicolors('value');
			if(colorpickerShowed !== false)
				colorpickerShowed.trigger("setColor",[cl,op]);
		}
	});
	
	//var $swatch = $input.next('.minicolors-swatch').hide();
	
	var $colorpicker = $input.next('.minicolors-panel').css({position:"absolute",zIndex:9999}).append($delPoint).hide();
	
	var c = { lastbg:"dumb solid color" };
	
	var selectedPoint = {
		transition: prefix.css+'transform 0.5s ease-in-out',
		transform:'scale(2) rotate(45deg)',
		opacity:1
	};
	
	var unselectedPoint = {
		transition: prefix.css+'transform 0.5s ease-in-out',
		transform:'scale(1) rotate(0deg)',
		opacity:0.7
	};
	
	var deg2rad = Math.PI/180;
	var rad2deg = 180/Math.PI;
	
	function mouse2left(ev,min,max){
		var bx = ev.pageX;
		var by = ev.pageY;
		
		var ax = pradius + c.pos.left;
		var ay = pradius + c.pos.top;
		
		var ab = Math.sqrt( Math.pow( bx - ax, 2 ) + Math.pow( by - ay, 2 ) );
		var ang = legalAngle(c.angle);
		
		var ac, left;
		 
		if(ang > 315 || ang < 45 || (ang > 135 && ang < 225 ) ){ // horizontal, +- 45°
			ac = Math.cos( deg2rad * ang ) * ab;
			if(bx < ax) ac = -ac;
		} else {
			ac = Math.sin( deg2rad * ang ) * ab; // vertical +- 45°
			if(by < ay) ac = -ac;
		}
		
		left = (ac + (barw/2)) - aradius;
		left = Math.max(Math.min(left,max),min);
		return parseInt(left,10);
	}
	
	$bar.on("click",function(e){
		if(colorpickerShowed !== false || Date.now() - lastDrag < 500) return;
		
		var max = barw - (c.pointPad + 1);
		var min = - ( c.pointPad + 1);
		var left = mouse2left(e,min,max);
		
		left -= parseInt( ($addPoint.width()-1)/2 ,10);
		var top = - parseInt( ( $addPoint.height() - $bar.height() - 1 )/2, 10 );
		
		//console.log(left);
		if(addPointShowed)
			$addPoint.animate({ left:left });
		else {
			addPointShowed = true;
			$addPoint.css({ left:left, top:top, zIndex:c.color.length }).show(200);
		}
	}).on("refresh",function(){
		$bar.trigger("set",[c.color]);
	}).on("add",function(e){
		var colors = c.color;
		var clr = c.color[c.color.length - 1];
		
		var pmax = barw - (c.pointPad + 1);
		var pmin = - ( c.pointPad + 1);
		var ratio = 100 / (pmax - pmin);
		
		var left = ($addPoint.css("left").slice(0,-2)*ratio) + (($addPoint.width()-1)/2);
		left = parseInt(left,10);
		
		clr = "#ff00ff "+left+"%";
		//console.log(ratio,clr);
		
		c.color.push(clr);
		$addPoint.hide(200);
		addPointShowed = false;
		$bar.trigger("set",[c.color]);
		$prev.trigger("refresh");
	}).on("set",function(e,colors){
		//console.log( colors );
		$addPoint.detach();
		$bar.empty();
		$addPoint.appendTo( $bar );
		c.points = [];
		
		var pmax = barw - (c.pointPad + 1);
		var pmin = - ( c.pointPad + 1);
		var ratio = 100 / (pmax - pmin);
		
		var ph = -3;
		var leftDec = c.pos.left + barx + c.pointPad + 1;
		var  topDec = c.pos.top + barx + c.pointPad + 1;
		
		var cx, cy;
		cx = leftDec + pradius;
		cy = topDec + pradius;
		//console.log(ratio);
		
		$.each(colors,function(n,t){
			var    css = this, color, opacity;
			if(css.charAt(0) === '#'){ // hex
				opacity = 1;
				var    grd = css.split(' ');
				     color = grd[0];
				var    pos = grd[1]; pos = parseInt(pos.toString().slice(0,-1),10);
			} else { // rgba & rgb
				var    grd = css.split(') ');
				var   rgba = grd[0];
				var    pos = grd[1]; pos = parseInt(pos.toString().slice(0,-1),10);
				
				var cl;

				if(rgba.charAt(3) === '(')
					cl = rgba.slice(4).split(','); // rgb
				else	cl = rgba.slice(5).split(','); // rgba
				
				var r = parseInt($.trim(cl[0]),10).toString(16); if(r.length < 2) r="0"+r;
				var g = parseInt($.trim(cl[1]),10).toString(16); if(g.length < 2) g="0"+g;
				var b = parseInt($.trim(cl[2]),10).toString(16); if(b.length < 2) b="0"+b;
				opacity = (cl[3] === undefined)?1:parseFloat($.trim(cl[3]),10);
				color = "#"+r+g+b;
			}
			  
 			//console.log(color + " "+ opacity);
			  
			var  point = {
				color : color,
				pos : pos,
				$ : 0,
				css : css,
				opacity:opacity
			};
			
			//console.log( point );
			
			var $point = $("<div/>").css({
				position:"absolute",
				left:Math.max(Math.min( parseInt(pos/ratio,10) - (c.pointPad + 1),pmax),pmin),
				top:ph,
				width:c.pointPad*2+1,
				height:c.pointPad*2+1,
				background:point.color,
				opacity:0.7,
				border:"1px solid black"
			}).on("getColor",function(){
			  	if(point.opacity == 1)
					return point.color;
				
				var cl = point.color.slice(1);
				var r = parseInt(cl.substring(0,2),16);
				var g = parseInt(cl.substring(2,4),16);
				var b = parseInt(cl.substring(4,6),16);
				
				return 'rgba('+r+','+g+','+b+','+parseFloat(point.opacity,10)+')';
			}).on("getCss",function(){
				return $point.triggerHandler("getColor")+" "+point.pos+"%";
			}).on("getPos",function(){
				var r = ratio;
				var a = 180 - ( c.angle-90 ) - 180;
				
				var left = parseInt($point.css('left').slice(0,-2)) + c.pointPad + 1;
				
				var l = Math.max(Math.min( parseInt(left*r,10),100),0);
				
				return l;
			}).on("setColor",function(e,color,alpha){
			//	console.log("set "+color+" "+alpha);
				point.color = color;
				point.opacity = (alpha===undefined)?1:alpha;
				
				var cssColor = $point.triggerHandler("getColor");
				var css = cssColor+" "+point.pos+"%";
				
				point.css = css;//$point.triggerHandler("getCss");// color+" "+point.pos+"%";
				$point.css('background',cssColor);
				
				$prev.trigger("sort");
				
				$prev.trigger("refresh");
			}).drag("start",function(ev,dd){
				
			}).drag(function(ev,dd){
				var left = mouse2left(ev,pmin,pmax);
				
				if(addPointShowed){
					$addPoint.hide(200);
					addPointShowed = false;
				}
				
				//$("#c").html('ab : '+ab+" ang:"+ang+" ac:"+ac+" left:"+left);
				
				$point.css({ left:left });
				
				var value = $point.triggerHandler("getPos");
				point.pos = value;
				point.css = $point.triggerHandler("getCss");//point.color+" "+value+"%";
				
				lastDrag = Date.now();
				
				$prev.trigger("sort").trigger("refresh");
				
				//c.color[n] = point.css;
				//$prev.trigger("refresh");
				//$("#a").html( value );
			}).on("click",function(e){
				//if(false !== colorpickerShowed && $point.is(colorpickerShowed)) return false;
				//$input.val(point.color);
				
			//	console.log("click "+point.color);
			  
				$point.css(selectedPoint);
				colorpickerShowed = $point;
				$colorpicker.show();

			//	console.log( useOpacity?point.opacity:1 );
			//	console.log(point.opacity);
				
				var op = useOpacity?point.opacity:1;
				
				$input.attr('data-opacity',op).minicolors('value',[point.color]);
				
				/*$input.minicolors('value',[point.color]); $input.minicolors('opacity',[op]);*/
				
				e.stopPropagation();
				
				//$prev.trigger("refresh");
			}).on("del",function(){
				//console.log("del")
				if(c.points.length < 3) return;
				var i = c.points.indexOf(point);
				//console.log(i);
				if(i !== -1){
					c.points[i].$.remove();
					colorpickerShowed = false;
					
				//	console.log(c.points);
					c.points.splice(i,1);
				//	console.log(c.points);
					//
					
					$colorpicker.hide();
					
					//$bar.trigger("set",[c.color]);
					$prev.trigger("sort");
					$prev.trigger("refresh");
				}
			}).appendTo($bar);
			
			//$point.colorpicker();
			
			point.$ = $point;
			point.css = $point.triggerHandler('getCss');
			//console.log(point.css);
			c.points.push(point);
		});
		
		$prev.trigger("sort");
	});
	
	$angle.on("set",function(e,a,refresh){
		c.angle = legalAngle(a);
		//console.log("set angle to "+c.angle);
		//a = ( (180 - a) / 360 ) * 2 * Math.PI;
		a = (180-a)*deg2rad;
		$angle.css({
		      top: pradius + Math.cos(a) * pradius - aradius,
		      left: pradius + Math.sin(a) * pradius - aradius
		});
		
		if(refresh === true) $prev.trigger("refresh");
	});
	
	$angle.drag("start",function(ev,dd){
		c.pos = $prev.offset();
	}).drag(function( ev, dd ){
		angle = Math.atan2( ev.pageX - ( pradius + c.pos.left ), ev.pageY - ( pradius + c.pos.top ) );
		var a = 180 - legalAngle(parseInt(rad2deg*angle,10));
		$angle.trigger("set",[a,true]);
	});
	
	$prev.on("sort",function(){
		c.points.sort(function(a,b) {
			if (a.pos < b.pos)
				return -1;
			if (a.pos > b.pos)
				return 1;
			return 0;
		});
		
		c.color = [];
		$.each(c.points,function(n,t){
			c.color.push( this.css );
			this.$.css("z-index",n);
		});
	}).on("css",function(){
		var a = c.angle;
		var ga = legalAngle(360-a);
		
		//console.log(a);
		
		switch(c.type){
			case "single":
				return { background:c.color };
			break;
			case "linear":
				return { background:prefix.css+'linear-gradient('+ga+'deg,'+c.color.join(",")+')',
					// background:'linear-gradient('+c.angle+'deg,'+c.color.join(",")+')'
				};
			break;
			case "radial":
				return { background:prefix.css+'radial-gradient('+ga+'deg,'+c.color.join(",")+')',
					// background:'radial-gradient('+c.angle+'deg,'+c.color.join(",")+')',
				};
			break;
		}
	}).on("refresh",function(e){
		//$prev.trigger("sort");
		
		var a = c.angle;
		var ga = 360-a;
		
		//$("#title").html(a);
		
		//var a = 360-c.angle;
		switch(c.type){
			case "single":
				$prev.css('background',c.color);
			break;
			case "linear":
				var colors = c.color.join(",");
				$prev.css('background',prefix.css+'linear-gradient('+ga+'deg,'+colors+')');
				//$prev.css('background','linear-gradient('+c.angle+'deg,'+colors+')');
				$bar.css('background',prefix.css+'linear-gradient(0deg,'+colors+')');
		//$bar.css({rotate:a}); // jquery transit
				//$bar.css('background','linear-gradient(90deg,'+colors+')');
				$bar.css(prefix.css+'transform','rotate('+a+'deg)');
				//console.log( c.angle-90 );
				
			break;
			case "radial":
				$prev.css('background',prefix.css+'radial-gradient('+ga+'deg,'+c.color.join(",")+')');
				//$prev.css('background','radial-gradient('+c.angle+'deg,'+c.color.join(",")+')');
			break;
		}
		
		//console.log( c.$t.triggerHandler("get") );
		
		var outcss = c.$t.triggerHandler("get",["css"]);

		if( c.lastbg === outcss.background ) return; c.lastbg = outcss.background; // dear, cpu

		c.$t.trigger("change",[c.$t.triggerHandler("get"),outcss]);
	});
	
	$.fn.radass = function(opt){
		
		var dopt = {
			type : "linear",
			angle : 90,
			color : [ "rgba(255, 0, 255, 1) 0%", "rgba(0, 255, 0, 0.7) 30%" , "rgba(255,255,255,1) 100%" ],
			radius : 32,
			pointPad : 4
		};
		
		opt = $.extend(dopt, opt);
		
		return this.each(function(){
			var $t = $(this);
			
			$t.on('click',function(e){
				//$t.trigger("load");
				//e.preventDefault();
			}).on("load",function(){
				c = opt;
				c.pos = $t.offset();
				c.$t = $t;
				$bar.trigger("set",[c.color]);
				$angle.trigger("set",[c.angle]).show();
			//	console.log( c.pos );
				$g.detach().hide().appendTo($t.parent().css({textAlign:"center"})).show().offset(c.pos); //"body")
			//console.log("ready!");
			}).on('set',function(e,grd){
				c.angle = grd.angle;
				c.color = grd.color;
				$t.trigger('load').trigger('refresh');
			}).on("get",function(ev,type){
				if(type === 'css')
					return $prev.triggerHandler("css");
				return {
					color : c.color,
					angle : c.angle,
					type : c.type
				};
			}).on("refresh",function(){
				//console.log("prev refresh");
				$prev.trigger("refresh");
				//console.log("ftw");
			}).on("gradhide",function(){
				$g.detach();
			}).on("gradshow",function(){
				var $p = $t.parent();
				//$p.css({background:"red"});
				//alert("zzz");
				$g.hide().appendTo($p).show();//"body").show();
			}).trigger("load").trigger("refresh");
		});
	};
	
	$(document).mouseup(function(e){
		
		if(addPointShowed === true && !$addPoint.is(e.target) && !$bar.is(e.target)){
			$addPoint.hide(200);
			addPointShowed = false;
			e.preventDefault();
		}
	  
		if(colorpickerShowed !== false && !$colorpicker.is(e.target) && $colorpicker.has(e.target).length === 0){
			$colorpicker.hide();
			colorpickerShowed.css(unselectedPoint);
			colorpickerShowed = false;
			e.preventDefault();
		} else {
			/*if(addPointShowed === true){
				$addPoint.hide();
				e.preventDefault();
			}*/
		}
	});
})(jQuery);
 
