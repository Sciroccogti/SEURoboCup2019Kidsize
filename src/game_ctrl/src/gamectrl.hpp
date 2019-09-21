#ifndef __GAMECTRL_HPP
#define __GAMECTRL_CPP

#include <QtWidgets>
#include <simulation/GameData.h>
#include <ros/ros.h>
#include <memory>
#include <game_ctrl/Score.h>
#include <game_ctrl/TeamInfo.h>

enum TeamColor
{
    TEAM_RED,
    TEAM_BLUE
};

class TeamLabel: public QWidget
{
Q_OBJECT
public:
    TeamLabel(QString name, TeamColor color);
    QString Name()
    {
        return nameLabel->text();
    }
    void SetScore(int s)
    {
        scoreLabel->setText(QString::number(s));
    }
    void SetName(QString name)
    {
        nameLabel->setText(name);
    }
private:
    QLabel *colorLabel;
    QLabel *nameLabel;
    QLabel *scoreLabel;
};

class StartDlg: public QDialog
{
Q_OBJECT
public:
    StartDlg(std::string cfg);
    QString redName, blueName;

public slots:
    void OnStart();

private:
    QComboBox *redTeamBox, *blueTeamBox;
    QPushButton *startBtn;
};

class GameCtrl: public QMainWindow
{
Q_OBJECT
public:
    GameCtrl(QString red, QString blue);

public slots:
    void OnFTimer();
    void OnSTimer();
    void OnBtnInitClicked();
    void OnBtnReadyClicked();
    void OnBtnPlayClicked();
    void OnBtnPauseClicked();
    void OnBtnFinishClicked();
    void OnBtnRedStandClicked();
    void OnBtnBlueStandClicked();

private:
    void ScoreUpdate(const game_ctrl::Score::ConstPtr &p);
    bool TeamInfoService(game_ctrl::TeamInfo::Request &req,
        game_ctrl::TeamInfo::Response &res);

private:
    QLabel *timeLabel, *stimeLabel, *stateLabel;
    TeamLabel *redTeam, *blueTeam;
    QTimer *fTimer, *sTimer;

    QPushButton *stateInitBtn, *stateReadyBtn, *statePlayBtn, *statePauseBtn, *stateFinishBtn;
    QPushButton *btnRedStand, *btnBlueStand;

    simulation::GameData mGdata;
    game_ctrl::Score mScore;

    std::shared_ptr<ros::NodeHandle> mNode;
    ros::Publisher mGdataPublisher;
    ros::Publisher mStandPublisher;
    ros::Subscriber mScoreSubscriber;
    ros::ServiceServer mTinfoServer;

    QString redName, blueName;
    bool paused;
    int mMs;
    int remainTime;
    const int totalTime = 60*10;
    const int mBasicTIme = 20;

    int readyRemainTime;
    const int readyTime = 10;
};


#endif
