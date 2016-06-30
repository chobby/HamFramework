//-----------------------------------------------
//
//	This file is part of the HamFramework for Siv3D.
//
//	Copyright (C) 2014-2016 Hamukun
//	Copyright (C) 2014-2016 Ryo Suzuki
//
//	Licensed under the MIT License.
//
//-----------------------------------------------

# pragma once
//# include "../Siv3D.hpp"
# include <Siv3D.hpp>

namespace ham
{
	
	class CameraController2DBase : public s3d::Uncopyable
	{
	public:

		CameraController2DBase()
		{
			m_windowSize = Window::Size();
			m_pos = m_windowSize * 0.5;
		}

		virtual void init() {}

		virtual void update() {}

		virtual void draw() const {}

		virtual void printCurrentState() const
		{
			Println(L"camera");
			Println(L"pos   : ", m_pos);
			Println(L"scale : ", m_scale);
		}

		void setPos(const Vec2& pos)
		{
			m_pos = pos;
		}

		const Vec2& getPos() const
		{
			return m_pos;
		}

		void setWindowSize(const Vec2& windowSize)
		{
			m_windowSize = windowSize;
		}

		const Vec2& getWindowSize() const
		{
			return m_windowSize;
		}

		void setScale(double scale)
		{
			m_scale = scale;
		}

		double getScale() const
		{
			return m_scale;
		}

		RectF getCameraArea() const
		{
			const Vec2 startPos = m_pos - m_windowSize * 0.5 / m_scale;
			const Vec2 size = m_windowSize / m_scale;
			return RectF(startPos, size);
		}

		virtual Mat3x2 generateCameraMatrix(const Vec2& pos, const Vec2& windowSize, double scale) const
		{
			return Mat3x2::Translate(-pos).scale(scale).translate(windowSize * 0.5);
		}

	protected:

		Vec2 m_windowSize;
		Vec2 m_pos;
		double m_scale = 1.0;
	};

	class ExampleCameraController2D : public CameraController2DBase
	{
	public:

		ExampleCameraController2D()
		{
			
		}


		void init() override
		{
			m_targetPos = m_pos;
			m_targetScale = m_scale;
		}

		void update() override
		{
			if (m_enableDefaultCameraManipulatorKeyboard)
			{
				manipulateCameraKeyboard();
			}
			if (m_enableDefaultCameramanipulatorMouse)
			{
				manipulateCameraMouse();
			}

			if (m_limitArea)
			{
				m_targetPos.x = Clamp(m_targetPos.x, m_limitArea.value().tl.x, m_limitArea.value().tr.x);
				m_targetPos.y = Clamp(m_targetPos.y, m_limitArea.value().tl.y, m_limitArea.value().bl.y);
			}

			m_pos = Lerp(m_pos, m_targetPos, m_lerpRatio);
			m_scale = Lerp(m_scale, m_targetScale, m_lerpRatio);
			m_speed = Lerp(m_speed, m_targetSpeed, m_lerpRatio);
		}

		void draw() const override
		{
			if (m_isDragging)
			{
				const double radius = 20.0;
				Circle(m_startDragScreenPos, radius).drawFrame(2.0, 2.0);
				if (m_deltaDragScreenPos.length() > radius)
					Line(m_startDragScreenPos + m_deltaDragScreenPos.normalized() * radius, m_startDragScreenPos + m_deltaDragScreenPos).drawArrow(4.0);
			}
		}

		void setTargetScale(double targetScale)
		{
			m_targetScale = targetScale;
		}

		double getTargetScale() const
		{
			return m_targetScale;
		}

		void setScaleRatio(double scaleRatio)
		{
			m_scaleRatio = scaleRatio;
		}

		double getScaleRatio() const
		{
			return m_scaleRatio;
		}

		void setTargetPos(const Vec2& targetPos)
		{
			m_targetPos = targetPos;
		}

		const Vec2& getTargetPos() const
		{
			return m_targetPos;
		}

		void setTargetSpeed(double targetSpeed)
		{
			m_targetSpeed = targetSpeed;
		}

		double getTargetSpeed() const
		{
			return m_targetSpeed;
		}

	protected:

		virtual void manipulateCameraKeyboard()
		{
			if (Input::KeyUp.pressed)
			{
				m_targetScale *= m_scaleRatio;
			}
			if (Input::KeyDown.pressed)
			{
				m_targetScale /= m_scaleRatio;
			}
			if (Input::KeyW.pressed)
			{
				m_targetPos += m_targetSpeed * Vec2::Up / m_targetScale;
			}
			if (Input::KeyA.pressed)
			{
				m_targetPos += m_targetSpeed * Vec2::Left / m_targetScale;
			}
			if (Input::KeyS.pressed)
			{
				m_targetPos += m_targetSpeed * Vec2::Down / m_targetScale;
			}
			if (Input::KeyD.pressed)
			{
				m_targetPos += m_targetSpeed * Vec2::Right / m_targetScale;
			}
		}

		virtual void manipulateCameraMouse()
		{
			if (Input::MouseR.clicked)
			{
				m_startDragScreenPos = Mouse::Pos();
				m_isDragging = true;
			}

			if (m_isDragging)
			{
				m_deltaDragScreenPos = Mouse::Pos() - m_startDragScreenPos;

				m_targetPos += m_mouseSpeedRatio * m_targetSpeed * m_deltaDragScreenPos / m_targetScale;

				if (Input::MouseR.released)
				{
					m_isDragging = false;
				}
			}

			if (Mouse::Wheel() < 0)
			{
				m_targetScale *= m_scaleRatio;
			}
			if (Mouse::Wheel() > 0)
			{
				m_targetScale /= m_scaleRatio;
			}
		}

		Vec2 m_targetPos;
		double m_targetScale = 1.0;
		double m_scaleRatio = 1.1;
		double m_lerpRatio = 0.2;

		double m_speed = 10.0;
		double m_targetSpeed = 10.0;

		Optional<RectF> m_limitArea;

		bool m_enableDefaultCameraManipulatorKeyboard = true;
		bool m_enableDefaultCameramanipulatorMouse = true;
		bool m_isDragging = false;

		Vec2 m_startDragScreenPos;
		Vec2 m_deltaDragScreenPos;

		double m_mouseSpeedRatio = 0.01;
	};

	class ExampleCameraControllerForBox2D : public ExampleCameraController2D
	{
	public:

		Mat3x2 generateCameraMatrix(const Vec2& pos, const Vec2& windowSize, double scale) const override
		{
			return Mat3x2::Translate(-pos).scale(scale, -scale).translate(windowSize * 0.5);
		}

	private:

		void manipulateCameraKeyboard() override
		{
			if (Input::KeyUp.pressed)
			{
				m_targetScale *= m_scaleRatio;
			}
			if (Input::KeyDown.pressed)
			{
				m_targetScale /= m_scaleRatio;
			}
			if (Input::KeyW.pressed)
			{
				m_targetPos += m_targetSpeed * Vec2::Down / m_targetScale;
			}
			if (Input::KeyA.pressed)
			{
				m_targetPos += m_targetSpeed * Vec2::Left / m_targetScale;
			}
			if (Input::KeyS.pressed)
			{
				m_targetPos += m_targetSpeed * Vec2::Up / m_targetScale;
			}
			if (Input::KeyD.pressed)
			{
				m_targetPos += m_targetSpeed * Vec2::Right / m_targetScale;
			}
		}

		void manipulateCameraMouse() override
		{
			if (Input::MouseR.clicked)
			{
				m_startDragScreenPos = Mouse::Pos();
				m_isDragging = true;
			}

			if (m_isDragging)
			{
				m_deltaDragScreenPos = Mouse::Pos() - m_startDragScreenPos;

				m_targetPos += m_mouseSpeedRatio * m_targetSpeed * Vec2(m_deltaDragScreenPos.x, -m_deltaDragScreenPos.y) / m_targetScale;

				if (Input::MouseR.released)
				{
					m_isDragging = false;
				}
			}

			if (Mouse::Wheel() < 0)
			{
				m_targetScale *= m_scaleRatio;
			}
			if (Mouse::Wheel() > 0)
			{
				m_targetScale /= m_scaleRatio;
			}
		}
	};

	class CameraManager2D
	{
	public:

		CameraManager2D()
		{
			m_cameraController = std::make_shared<ExampleCameraController2D>();
		}

		void init()
		{
			m_cameraController->init();
		}

		void update()
		{
			m_cameraController->update();
		}

		void begin() const 
		{
			m_beforeMatrix = Graphics2D::GetTransform();
			const auto cameraMatrix = m_cameraController->generateCameraMatrix(m_cameraController->getPos(), m_cameraController->getWindowSize(), m_cameraController->getScale());
			const auto afterMatrix = cameraMatrix * m_beforeMatrix;
			Graphics2D::SetTransform(afterMatrix);
			Mouse::SetTransform(afterMatrix);			
		}

		void end() const
		{
			Graphics2D::SetTransform(m_beforeMatrix);
			Mouse::SetTransform(m_beforeMatrix);
		}

		void draw() const
		{
			m_cameraController->draw();
		}	

		void printCurrentState() const
		{
			m_cameraController->printCurrentState();
		}

		void setCameraController(const std::shared_ptr<CameraController2DBase>& cameraController)
		{
			m_cameraController = cameraController;
		}

		const std::shared_ptr<CameraController2DBase>& getCameraController() const
		{
			return m_cameraController;
		}

	private:

		std::shared_ptr<CameraController2DBase> m_cameraController;

		mutable Mat3x2 m_beforeMatrix;

	};
	
}
