(function (global, factory) {
    typeof exports === 'object' && typeof module !== 'undefined' ? module.exports = factory() :
        typeof define === 'function' && define.amd ? define(factory) :
            (global.MoveLine = factory());
}(this, (function () {
    'use strict';




    var global = typeof window === 'undefined' ? {} : window;

    var requestAnimationFrame = global.requestAnimationFrame || global.mozRequestAnimationFrame || global.webkitRequestAnimationFrame || global.msRequestAnimationFrame || function (callback) {
        return global.setTimeout(callback, 1000 / 60);
    };
    var MoveLine = function MoveLine(userOptions) {
        var self = this;

        //默认参数
        var options = {
            //marker点半径
            markerRadius: 5,
            //marker点颜色,为空或null则默认取线条颜色
            markerColor: '#ff0808',
            //线条类型 solid、dashed、dotted
            lineType: 'solid',
            //线条宽度
            lineWidth: 2,
            //线条颜色
            colors: ['#F9815C', '#F8AB60', '#EDCC72', '#E2F194', '#94E08A', '#4ECDA5'],
            //移动点半径
            moveRadius: 5,
            //移动点颜色
            fillColor: '#ff0000',
            //移动点阴影颜色
            shadowColor: '#ffff00',
            //移动点阴影大小
            shadowBlur: 5,

            mapWidth: 1024,
            mapHeight: 768
        };

        //全局变量
        var baseLayer = null,
            animationLayer = null,
            //width = options.mapWidth,
            //height = options.mapHeight,
            animationFlag = true,
            markLines = [];

        //参数合并
        var merge = function merge(userOptions, options) {
            Object.keys(userOptions).forEach(function (key) {
                options[key] = userOptions[key];
            });
        };

        function Marker(opts) {
            this.city = opts.city;
            this.location = opts.location;
            this.color = opts.color;

        }

        Marker.prototype.draw = function (context) {
            var pixel = this.pixel = this.location;

            context.save();
            context.beginPath();
            context.fillStyle = options.markerColor || this.color;
            context.arc(pixel.x, pixel.y, options.markerRadius, 0, Math.PI * 2, true);
            context.closePath();
            context.fill();

            context.textAlign = 'center';
            context.textBaseline = 'middle';
            context.font = '12px Microsoft YaHei';
            context.fillStyle = options.markerColor || this.color;
            context.fillText(this.city, pixel.x, pixel.y - 10);
            context.restore();
        };

        function MarkLine(opts) {
            this.from = opts.from;
            this.to = opts.to;
            this.id = opts.id;
            this.ref_points = opts.ref_points;
            this.step = 0;

        }



        MarkLine.prototype.drawMarker = function (context) {
            this.from.draw(context);
            this.to.draw(context);
        };

        MarkLine.prototype.drawLinePath = function (context) {
            //var pointList = this.path = this.getPointList(this.from.location, this.to.location);

            var pointList = this.path = this.ref_points;
            var len = pointList.length;
            context.save();
            context.beginPath();
            context.lineWidth = options.lineWidth;
            context.strokeStyle = options.colors[this.id];

            if (!options.lineType || options.lineType == 'solid') {
                context.moveTo(pointList[0][0], pointList[0][1]);
                for (var i = 0; i < len; i++) {
                    context.lineTo(pointList[i][0], pointList[i][1]);
                }
            } else if (options.lineType == 'dashed' || options.lineType == 'dotted') {
                for (var i = 1; i < len; i += 2) {
                    context.moveTo(pointList[i - 1][0], pointList[i - 1][1]);
                    context.lineTo(pointList[i][0], pointList[i][1]);
                }
            }
            context.stroke();
            context.restore();
            this.step = 0; //缩放地图时重新绘制动画
        };

        MarkLine.prototype.drawMoveCircle = function (context) {
            //var pointList = this.path || this.getPointList(this.from.location, this.to.location);
            var pointList = this.path || this.ref_points;

            context.save();
            context.fillStyle = options.fillColor;
            context.shadowColor = options.shadowColor;
            context.shadowBlur = options.shadowBlur;
            context.beginPath();
            context.arc(pointList[this.step][0], pointList[this.step][1], options.moveRadius, 0, Math.PI * 2, true);
            context.fill();
            context.closePath();
            context.restore();
            this.step += 1;
            if (this.step >= pointList.length) {
                this.step = 0;
            }
        };

        //底层canvas渲染，标注，线条
        var brush = function brush() {
            var baseCtx = baseLayer.getContext('2d');
            if (!baseCtx) {
                return;
            }

            addMarkLine();

            baseCtx.clearRect(0, 0, options.mapWidth, options.mapHeight);
            //console.log(options.mapWidth + '--' + options.mapHeight);

            markLines.forEach(function (line) {
                line.drawMarker(baseCtx);
                line.drawLinePath(baseCtx);
            });
        };

        //上层canvas渲染，动画效果
        var render = function render() {
            var animationCtx = animationLayer.getContext('2d');
            if (!animationCtx) {
                return;
            }

            if (!animationFlag) {
                animationCtx.clearRect(0, 0, options.mapWidth, options.mapHeight);
                return;
            }

            animationCtx.fillStyle = 'rgba(0,0,0,.93)';
            var prev = animationCtx.globalCompositeOperation;
            animationCtx.globalCompositeOperation = 'destination-in';
            animationCtx.fillRect(0, 0, options.mapWidth, options.mapHeight);
            animationCtx.globalCompositeOperation = prev;

            for (var i = 0; i < markLines.length; i++) {
                var markLine = markLines[i];
                markLine.drawMoveCircle(animationCtx); //移动圆点
            }
        };


        var addMarkLine = function addMarkLine() {
            markLines = [];
            if (options.data) {
                options.data.forEach(function (line, i) {
                    var points = [];
                    var icount = 0;
                    for (var key in line['ref_points']) {
                        icount++;
                        if (icount > 3) {
                            icount = 0;
                            points.push([line['ref_points'][key]['x'] / Scaling - x_offset, y_offset - line['ref_points'][key]['y'] / Scaling]);
                        }

                    }
                    // var arr = Object.values(line.ref_points); //对象转化为数组
                    // console.log(arr);
                    markLines.push(new MarkLine({
                        id: i,

                        from: new Marker({
                            city: line.from,
                            location: { x: points[0][0], y: points[0][1] },
                            color: options.colors[i]

                        }),
                        to: new Marker({
                            city: line.to,
                            location: { x: points[points.length - 1][0], y: points[points.length - 1][1] },
                            color: options.colors[i]
                        }),
                        ref_points: points
                    }));
                });
            }
        };

        //初始化
        var init = function init(options) {
            merge(userOptions, options);

            // baseLayer = new CanvasLayer({
            //     map: map,
            //     update: brush
            // });

            // animationLayer = new CanvasLayer({
            //     map: map,
            //     update: render
            // });
            // baseLayer = document.createElement('canvas');
            // baseLayer.id = 'canvas11';
            // //var ctx = baseLayer.getContext('2d');
            // baseLayer.style.cssText = 'position:absolute;' + 'left:0;' + 'top:0;' + 'z-index:0;';

            // baseLayer.width = options.mapWidth;
            // baseLayer.height = options.mapHeight;
            // baseLayer.style.width = baseLayer.width + 'px';
            // baseLayer.style.height = baseLayer.height + 'px';
            // map.appendChild(baseLayer);

            // brush();

            // animationLayer = document.createElement('canvas');
            // animationLayer.id = 'canvas22';
            // //var ctx = baseLayer.getContext('2d');
            // animationLayer.style.cssText = 'position:absolute;' + 'left:0;' + 'top:0;' + 'z-index:0;';
            // animationLayer.width = options.mapWidth;
            // animationLayer.height = options.mapHeight;
            // animationLayer.style.width = animationLayer.width + 'px';
            // animationLayer.style.height = animationLayer.height + 'px';
            // map.appendChild(animationLayer);

            // render();
            //




            // 这里绘制春里的planning路径
            //baseLayer = document.getElementById('canvas_refline');
            baseLayer = document.getElementById('canvas_planning');
            brush();

            animationLayer = document.getElementById('canvas_refline_move');
            render();


            (function drawFrame() {
                requestAnimationFrame(drawFrame);
                render();
            })();
        };

        init(options);

        self.options = options;

        MoveLine.prototype.update = function (resetOpts) {
            for (var key in resetOpts) {
                this.options[key] = resetOpts[key];
            }
            brush();
        };

    };


    return MoveLine;

})));
