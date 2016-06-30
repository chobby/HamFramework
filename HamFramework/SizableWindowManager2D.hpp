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
	class SizableWindowmanager2D
	{
	public:

		SizableWindowmanager2D(const Vec2& basicWindowSize)
			: m_basicWindowSize(basicWindowSize)
		{

		}

		SizableWindowmanager2D()
		{
			m_basicWindowSize = Window::Size();
		}

		void begin() const
		{
			const bool widthIsLongerThanHeight = Window::Width() > Window::Height();
			const double scale = widthIsLongerThanHeight ? Window::Height() / m_basicWindowSize.y : Window::Width() / m_basicWindowSize.x;

			//const double aspectRatio = m_basicWindowSize.x / m_basicWindowSize.y;
			const double extendedAspectRatio = static_cast<double>(Window::Width()) / Window::Height();

			if (widthIsLongerThanHeight)
			{				
				m_extendedWindowSize = Vec2(m_basicWindowSize.y * extendedAspectRatio, m_basicWindowSize.y);
			}
			else
			{
				m_extendedWindowSize = Vec2(m_basicWindowSize.x, m_basicWindowSize.x / extendedAspectRatio);
			}

			m_beforeBeginMatrix = Graphics2D::GetTransform();
			const auto afterMatrix = Mat3x2::Translate(-m_basicWindowSize * 0.5).scale(scale).translate(Window::Center());
			Graphics2D::SetTransform(afterMatrix);
			Mouse::SetTransform(afterMatrix);
		}
		
		void end() const
		{
			Graphics2D::SetTransform(m_beforeBeginMatrix);
			Mouse::SetTransform(m_beforeBeginMatrix);
		}

		void identityTransformBegin() const
		{
			m_beforeIdentityMatrix = Graphics2D::GetTransform();
			Graphics2D::SetTransform(Mat3x2::Identity());
		}

		void identityTransformEnd() const
		{
			Graphics2D::SetTransform(m_beforeIdentityMatrix);
		}

		const Vec2& getBasicWindowSize() const
		{
			return m_basicWindowSize;
		}

		const Vec2& getExtendedWindowSize() const
		{
			return m_extendedWindowSize;
		}

	private:

		Vec2 m_basicWindowSize;
		mutable Vec2 m_extendedWindowSize;
		mutable Mat3x2 m_beforeBeginMatrix; 
		mutable Mat3x2 m_beforeIdentityMatrix;
		
	};
}
