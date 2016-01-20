#ifndef GLWINDOW2_H
#define GLWINDOW2_H

#include <cvd/glwindow.h>
#include <TooN/TooN.h>

class GLWindow2 : public CVD::GLWindow, public CVD::GLWindow::EventHandler
{
public:
    GLWindow2(CVD::ImageRef irSize, std::string sTitle);

    // Getter/setter fns
    std::pair<TooN::Vector<6>, TooN::Vector<6> > GetPoseUpdate();

    // Functions to set up window
    void SetupViewport(const CVD::ImageRef irPos, const CVD::ImageRef irSize);
    void SetupOrthoProj(const TooN::Vector<4> &v4);
    void SetupPerspectiveProj(const TooN::Vector<4> &v4);
    void SetupWindowOrtho(const CVD::ImageRef irSize);
    void SetupVideoRasterPosAndZoom();

    // Event handlers
    void HandlePendingEvents();

private:
    // Event handlers
    virtual void on_key_down(GLWindow&, int);
    virtual void on_mouse_move(GLWindow&, CVD::ImageRef, int);
    CVD::ImageRef irMouseLastPos;

    CVD::ImageRef irVideoSourceSize;

    // Updates to view point in GLWindow
    TooN::Vector<6> v6PoseUpdate;
    TooN::Vector<6> v6MCUpdate;
};

#endif // GLWINDOW2_H
