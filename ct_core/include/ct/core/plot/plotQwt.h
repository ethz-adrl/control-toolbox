
#pragma once

#include <qwt/qwt_plot_curve.h>
#include <qwt/qwt_plot.h>
#include <qwt/qwt_plot_magnifier.h>
#include <qwt/qwt_plot_panner.h>
#include <qwt/qwt_plot_picker.h>
#include <qwt/qwt_picker_machine.h>
#include <qwt/qwt_legend.h>
#include <qapplication.h>

namespace ct {
namespace core {
namespace plotQwt {

class Figure
{
public:
    Figure() : holdOn_(false), magnifier_(plot_.canvas()), panner_(plot_.canvas()), picker_(plot_.canvas())
    {
        plot_.setFixedWidth(800);
        plot_.setFixedHeight(600);
        plot_.setCanvasBackground(QBrush(Qt::white));

        picker_.setStateMachine(new QwtPickerDragPointMachine);
        picker_.setTrackerMode(QwtPicker::AlwaysOn);
        picker_.setEnabled(true);
        picker_.setRubberBandPen(QColor(Qt::black));
        picker_.setRubberBand(QwtPicker::CrossRubberBand);
        //picker_.setTrackerPen(QPen(QColor(255,0,0)));
    }

    void plot(const std::vector<double>& y)
    {
        std::cout << "plotting" << std::endl;

        if (!holdOn_ || curves_.size() == 0)
            createCurve();

        Eigen::VectorXd x;
        x.setLinSpaced(y.size(), 0, y.size());

        curves_.back()->setSamples(x.data(), y.data(), x.size());

        std::cout << "set data" << std::endl;
    }

    void draw()
    {
        std::cout << "showing result" << std::endl;
        plot_.replot();
        plot_.show();
        std::cout << "done" << std::endl;
    }

    void hold(bool hold = true) { holdOn_ = hold; }
    void title(const std::string& title) { plot_.setWindowTitle(title.c_str()); }
    void setDimensions(size_t dimX_pixels, size_t dimY_pixels)
    {
        plot_.setFixedWidth(dimX_pixels);
        plot_.setFixedHeight(dimY_pixels);
    }

private:
    typedef std::shared_ptr<QwtPlotCurve> QwtPlotCurvePtr;

    void setDefaultCurveParams(QwtPlotCurvePtr& curve) { curve->setRenderHint(QwtPlotItem::RenderAntialiased); }
    void createCurve()
    {
        std::cout << "creating curve" << std::endl;

        // create new curve
        curves_.resize(curves_.size() + 1);
        curves_.back() = QwtPlotCurvePtr(new QwtPlotCurve);
        setDefaultCurveParams(curves_.back());

        curves_.back()->attach(&plot_);
    }

    bool holdOn_;

    QwtPlot plot_;
    QwtPlotMagnifier magnifier_;
    QwtPlotPanner panner_;
    QwtPlotPicker picker_;
    std::vector<QwtPlotCurvePtr> curves_;
};

typedef std::shared_ptr<Figure> FigurePtr;

namespace detail {


struct _application
{
public:
    FigurePtr createFigure()
    {
        std::cout << "creating figure" << std::endl;

        figures_.resize(figures_.size() + 1);
        figures_.back() = FigurePtr(new Figure());
        figures_.back()->title("Figure " + std::to_string(figures_.size() - 1));

        return figures_.back();
    }

    _application()
    {
        std::cout << "initializing application " << std::endl;

        fake_argv_[0] = "foo";
        fake_argv_[1] = NULL;

        application_ = new QApplication(fake_argc_, fake_argv_);
    }


    ~_application() { delete application_; }
    void exec() { application_->exec(); }
private:
    std::string appName_ = "Plot";
    static const int fake_argc_size_ = 2;
    int fake_argc_ = fake_argc_size_;
    char* fake_argv_[fake_argc_size_];

    QApplication* application_;

    std::vector<FigurePtr> figures_;
};

static void _startApplication(_application*& app)
{
    std::cout << "starting application" << std::endl;
    app = new _application();
    std::cout << "created application, will now exec" << std::endl;
    app->exec();
}

struct _interpreter
{
    static _application* get()
    {
        static _interpreter ctx;
        if (ctx.app_ == nullptr)
        {
            std::cout << "ctx.app_ is null ptr" << std::endl;
            exit(-1);
        }
        return ctx.app_;
    }

private:
    _interpreter() : app_(nullptr)
    {
        //	  std::cout << "initializing interpreter" << std::endl;
        //	  applicationThread_ = new std::thread(  [this] {_startApplication(this->app_); });
        //
        //	  // hacky: wait for initialization
        //	  while(app_ == nullptr)
        //	  {
        //		  std::this_thread::sleep_for(std::chrono::milliseconds(10));
        //	  }
        //
        //	  std::cout << "interpreter init done " <<std::endl;
        app_ = new _application();
    }

    _interpreter(const _interpreter& other) = delete;

    ~_interpreter()
    {
        std::cout << "waiting for app to finish" << std::endl;
        //	  applicationThread_->join();
        //	  delete applicationThread_;
        delete app_;
    }

    //std::thread* applicationThread_;
    _application* app_;
};


}  // namespace detail

FigurePtr createFigure()
{
    return detail::_interpreter::get()->createFigure();
}

void render()
{
    return detail::_interpreter::get()->exec();
}
}
}
}
