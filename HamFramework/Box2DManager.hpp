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

# include <Siv3D.hpp>
# include <Siv3DAddon\Box2D.hpp>

namespace ham
{
	Vec2 B2Vec2ToVec2(const b2Vec2& b2Vec2)
	{
		return Vec2(b2Vec2.x, b2Vec2.y);
	}

	b2Vec2 Vec2ToB2Vec2(const Vec2& vec2)
	{
		return b2Vec2(static_cast<float32>(vec2.x), static_cast<float32>(vec2.y));
	}

	class Box2DBody;

	class Box2DShape
	{
	public:

		using Box2DBodyPtr = std::weak_ptr<Box2DBody>;

		Box2DShape()
		{

		}

		using FixtureDefinition = b2FixtureDef;

		void setDensity(double density)
		{
			m_fixtureDef.density = static_cast<float32>(density);
		}

		void setFriction(double friction)
		{
			m_fixtureDef.friction = static_cast<float32>(friction);
		}

		void setRestitution(double restitution)
		{
			m_fixtureDef.restitution = static_cast<float32>(restitution);
		}

		const FixtureDefinition& getFixtureDef() const
		{
			return m_fixtureDef;
		}

		void setColor(const Color& color)
		{
			m_color = color;
		}

		void setFilterGroupIndex(int groupIndex)
		{
			m_fixtureDef.filter.groupIndex = static_cast<int16>(groupIndex);
		}

		virtual void draw(const Box2DBodyPtr& body) const = 0;

	protected:

		FixtureDefinition m_fixtureDef;

		Color m_color = Palette::White;

	};

	class Box2DInitialBodyStatus
	{
	public:

		using BodyDefinition = b2BodyDef;

		void setBodyType(b2BodyType type)
		{
			m_bodyDef.type = type;
		}

		virtual void setPos(const Vec2& pos)
		{
			m_bodyDef.position = Vec2ToB2Vec2(pos);
		}

		void setAngularDamping(double anglarDamping)
		{
			m_bodyDef.angularDamping = static_cast<float32>(anglarDamping);
		}

		const BodyDefinition& getBodyDefinition() const
		{
			return m_bodyDef;
		}

		void setAngle(double angle)
		{
			m_bodyDef.angle = static_cast<float32>(angle);
		}

		void setBullet(bool flag)
		{
			m_bodyDef.bullet = flag;
		}

	private:

		BodyDefinition m_bodyDef;

	};


	class Box2DBody : public std::enable_shared_from_this<Box2DBody>
	{
	public:

		using WorldWeakPtr = std::weak_ptr<b2World>;
		using BodyPtr = b2Body*;
		using InitialBodyStatusPtr = std::shared_ptr<Box2DInitialBodyStatus>;
		using ShapePtr = std::shared_ptr<Box2DShape>;
		using Box2DBodyPtr = std::shared_ptr<Box2DBody>;

		Box2DBody(const WorldWeakPtr& world)
			: m_world(world)
		{
			m_initialBodyStatus = std::make_shared<Box2DInitialBodyStatus>();
		}

		void init()
		{
			myPtr = shared_from_this();

			createBody();
		}

		virtual ~Box2DBody()
		{
			if(!m_world.expired())
				m_world.lock()->DestroyBody(m_body);
		}

		const BodyPtr& getBodyPtr() const
		{
			return m_body;
		}

		virtual void draw() const
		{
			for (const auto& fixtureDef : m_fixtureDefinitions)
			{
				fixtureDef->draw(myPtr);
			}
		}

		void setShape(const ShapePtr& shape)
		{
			m_fixtureDefinitions.push_back(shape);
		}

		void setTransform(const Vec2& position, double angle)
		{
			m_body->SetTransform(Vec2ToB2Vec2(position), static_cast<float32>(angle));
		}

		void setAwake(bool awake)
		{
			m_body->SetAwake(awake);
		}

		const InitialBodyStatusPtr& getInitialBodyStatusPtr() const
		{
			return m_initialBodyStatus;
		}

		void setInitialBodyStatusPtr(const InitialBodyStatusPtr& initialBodyStatus)
		{
			m_initialBodyStatus = initialBodyStatus;
		}



	protected:		

		virtual void createBody()
		{
			m_body = m_world.lock()->CreateBody(&m_initialBodyStatus->getBodyDefinition());

			for (const auto& fixtureDef : m_fixtureDefinitions)
			{
				m_body->CreateFixture(&fixtureDef->getFixtureDef());
			}				
		}

		Box2DBodyPtr myPtr;

		WorldWeakPtr m_world;

		BodyPtr m_body;

		InitialBodyStatusPtr m_initialBodyStatus;

		Array<ShapePtr> m_fixtureDefinitions;

	};

	class Box2DLine : public Box2DShape
	{
	public:

		Box2DLine()
			: Box2DShape()
		{
			m_fixtureDef.shape = &m_edgeShape;
		}

		using EdgeShape = b2EdgeShape;

		void draw(const Box2DBodyPtr& body) const override
		{
			const double angle = body.lock()->getBodyPtr()->GetAngle();
			const Vec2 pos = B2Vec2ToVec2(body.lock()->getBodyPtr()->GetPosition());

			const Vec2 start = pos + m_line.begin.rotated(angle);
			const Vec2 end = pos + m_line.end.rotated(angle);

			Line(start, end).draw(0.1, m_color);
		}

		void setLine(const Line& line)
		{
			const b2Vec2 start = Vec2ToB2Vec2(line.begin);
			const b2Vec2 end = Vec2ToB2Vec2(line.end);

			m_edgeShape.Set(start, end);

			m_line = line;
		}

	private:

		EdgeShape m_edgeShape;

		Line m_line;

	};

	class Box2DLineString : public Box2DShape
	{
	public:

		using ChainShape = b2ChainShape;

		Box2DLineString()
			: Box2DShape()
		{
			m_fixtureDef.shape = &m_chainShape;
		}

		void draw(const Box2DBodyPtr& body) const override
		{
			const double angle = body.lock()->getBodyPtr()->GetAngle();
			const Vec2 pos = B2Vec2ToVec2(body.lock()->getBodyPtr()->GetPosition());

			Array<Vec2> points;

			for (const auto& point : m_lineString.getArray())
			{
				points.push_back(pos + point.rotated(angle));
			}

			LineString(points).draw(0.1, m_color, m_isClosedCurve);
		}

		void setLineStringOpen(const LineString& lineString)
		{
			Array<b2Vec2> points;

			for (const auto& point : lineString.getArray())
			{
				points.push_back(Vec2ToB2Vec2(point));
			}			

			m_lineString = lineString;

			m_chainShape.CreateChain(points.data(), points.size());

			m_isClosedCurve = false;
		}

		void setLineStringLoop(const LineString& lineString)
		{
			Array<b2Vec2> points;

			for (const auto& point : lineString.getArray())
			{
				points.push_back(Vec2ToB2Vec2(point));
			}

			m_lineString = lineString;

			m_chainShape.CreateLoop(points.data(), points.size());

			m_isClosedCurve = true;
		}

	private:
		


		ChainShape m_chainShape;

		LineString m_lineString;

		bool m_isClosedCurve = false;

	};

	class Box2DRect : public Box2DShape
	{
	public:

		using PolygonShape = b2PolygonShape;

		Box2DRect()
			: Box2DShape()
		{
			m_fixtureDef.shape = &m_polygonShape;
		}

		void draw(const Box2DBodyPtr& body) const override
		{
			const double angle = body.lock()->getBodyPtr()->GetAngle();
			const Vec2 pos = B2Vec2ToVec2(body.lock()->getBodyPtr()->GetPosition());

			RectF(width, height).setCenter(pos).rotated(angle).draw(m_color);
		}

		void setSize(const Vec2& size)
		{
			width = static_cast<float32>(size.x);

			height = static_cast<float32>(size.y);

			m_polygonShape.SetAsBox(width * 0.5f, height * 0.5f);			
		}

	private:

		PolygonShape m_polygonShape;

		float32 width = 100.0f;

		float32 height = 20.0f;

	};

	class Box2DCircle : public Box2DShape
	{
	public:

		using CircleShape = b2CircleShape;

		Box2DCircle()
			: Box2DShape()
		{
			m_fixtureDef.shape = &m_circleShape;
		}

		void draw(const Box2DBodyPtr& body) const override
		{
			const double angle = body.lock()->getBodyPtr()->GetAngle();
			const Vec2 pos = B2Vec2ToVec2(body.lock()->getBodyPtr()->GetPosition());

			Circle(pos, m_circle.r).draw(m_color);
		}

		void setRadius(double radius)
		{			
			m_circleShape.m_radius = static_cast<float32>(radius);

			m_circle.r = radius;
		}

	private:

		Circle m_circle;

		CircleShape m_circleShape;

	};

	class Box2DPolygon : public Box2DShape
	{
	public:

		using PolygonShape = b2PolygonShape;

		Box2DPolygon()
			: Box2DShape()
		{
			m_fixtureDef.shape = &m_polygonShape;
		}

		void setPolygon(const Polygon& polygon)
		{
			Array<b2Vec2> vertices;

			for (const auto& point : polygon.outer())
			{
				vertices.push_back(Vec2ToB2Vec2(point));
			}

			m_polygonShape.Set(vertices.data(), vertices.size());

			m_polygon = polygon;
		}

		void draw(const Box2DBodyPtr& body) const override
		{
			const double angle = body.lock()->getBodyPtr()->GetAngle();
			const Vec2 pos = B2Vec2ToVec2(body.lock()->getBodyPtr()->GetPosition());

			m_polygon.rotated(angle).movedBy(pos).draw(m_color);
		}

	private:

		PolygonShape m_polygonShape;

		Polygon m_polygon;

	};

	class Box2DInitialWheelJointStatus
	{
	public:

		using WheelJointDef = b2WheelJointDef;
		using Box2DBodyPtr = std::shared_ptr<Box2DBody>;

		void setMotorSpeed(double motorSpeed)
		{
			m_wheelJointDef.motorSpeed = static_cast<float32>(motorSpeed);
		}

		void setMaxMotorTorque(double maxMotorTorque)
		{
			m_wheelJointDef.maxMotorTorque = static_cast<float32>(maxMotorTorque);
		}

		void setFrequencyHz(double frequencyHz)
		{
			m_wheelJointDef.frequencyHz = static_cast<float32>(frequencyHz);
		}

		void setDampingRatio(double dampingRatio)
		{
			m_wheelJointDef.dampingRatio = static_cast<float32>(dampingRatio);
		}

		void enableMotor(bool flag)
		{
			m_wheelJointDef.enableMotor = flag;
		}

		void init(const Box2DBodyPtr& bodyA, const Box2DBodyPtr& bodyB, const Vec2& anchor, const Vec2& axis)
		{
			m_wheelJointDef.Initialize(bodyA->getBodyPtr(), bodyB->getBodyPtr(), Vec2ToB2Vec2(anchor), Vec2ToB2Vec2(axis));
		}

		const WheelJointDef& getWheelJointDefinition() const
		{
			return m_wheelJointDef;
		}

	private:

		WheelJointDef m_wheelJointDef;

	};

	class Box2DWheelJoint
	{
	public:

		using WorldWeakPtr = std::weak_ptr<b2World>;
		using InitialWheelJointStatusPtr = std::shared_ptr<Box2DInitialWheelJointStatus>;
		using WheelJointPtr = b2WheelJoint*;


		Box2DWheelJoint(const WorldWeakPtr& world)
			: m_world(world)
		{

		}

		virtual ~Box2DWheelJoint()
		{
			if(!m_world.expired())
				m_world.lock()->DestroyJoint(m_wheelJoint);
		}

		void setInitialWheelJointStatus(const InitialWheelJointStatusPtr& initialWheelJointStatus)
		{
			m_initialWheelJointStatus = initialWheelJointStatus;
		}

		void setMotorSpeed(double motorSpeed)
		{
			m_wheelJoint->SetMotorSpeed(static_cast<float32>(motorSpeed));
		}

		void init()
		{
			m_wheelJoint = static_cast<WheelJointPtr>(m_world.lock()->CreateJoint(&m_initialWheelJointStatus->getWheelJointDefinition()));
		}
		
	private:

		WorldWeakPtr m_world;

		InitialWheelJointStatusPtr m_initialWheelJointStatus;

		WheelJointPtr m_wheelJoint;

	};

	class Box2DInitialRevoluteJointStatus
	{
	public:

		using RevoluteJointDef = b2RevoluteJointDef;
		using Box2DBodyPtr = std::shared_ptr<Box2DBody>;

		void init(const Box2DBodyPtr& bodyA, const Box2DBodyPtr& bodyB, const Vec2& anchor)
		{
			m_revoluteJointDef.Initialize(bodyA->getBodyPtr(), bodyB->getBodyPtr(), Vec2ToB2Vec2(anchor));
		}

		void setBodyA(const Box2DBodyPtr& bodyA)
		{
			m_revoluteJointDef.bodyA = bodyA->getBodyPtr();
		}

		void setBodyB(const Box2DBodyPtr& bodyB)
		{
			m_revoluteJointDef.bodyB = bodyB->getBodyPtr();
		}

		void setLowerAngle(double lowerAngle)
		{
			m_revoluteJointDef.lowerAngle = static_cast<float32>(lowerAngle);
		}

		void setUpperAngle(double upperAngle)
		{
			m_revoluteJointDef.upperAngle = static_cast<float32>(upperAngle);
		}

		void setMotorSpeed(double motorSpeed)
		{
			m_revoluteJointDef.motorSpeed = static_cast<float32>(motorSpeed);
		}

		void setLocalAnchorA(const Vec2& anchor)
		{
			m_revoluteJointDef.localAnchorA = Vec2ToB2Vec2(anchor);
		}

		void setLocalAnchorB(const Vec2& anchor)
		{
			m_revoluteJointDef.localAnchorB = Vec2ToB2Vec2(anchor);
		}

		void setEnableMotoor(bool flag)
		{
			m_revoluteJointDef.enableMotor = flag;
		}

		void setEnableLimit(bool flag)
		{
			m_revoluteJointDef.enableLimit = flag;
		}

		void setMaxMotorTorque(double maxTorque)
		{
			m_revoluteJointDef.maxMotorTorque = static_cast<float32>(maxTorque);
		}

		void setCollideConnected(bool flag)
		{
			m_revoluteJointDef.collideConnected = flag;
		}

		const RevoluteJointDef& getRevoluteJointDefinition() const
		{
			return m_revoluteJointDef;
		}

	private:

		RevoluteJointDef m_revoluteJointDef;

	};

	class Box2DRevoluteJoint
	{
	public:
		using WorldWeakPtr = std::weak_ptr<b2World>;
		using InitialRevoluteJointStatusPtr = std::shared_ptr<Box2DInitialRevoluteJointStatus>;
		using RevoluteJointPtr = b2RevoluteJoint*;


		Box2DRevoluteJoint(const WorldWeakPtr& world)
			: m_world(world)
		{

		}

		virtual ~Box2DRevoluteJoint()
		{
			if (!m_world.expired())
				m_world.lock()->DestroyJoint(m_revoluteJoint);
		}

		void setInitialRevoluteJointStatus(const InitialRevoluteJointStatusPtr& initialRevoluteJointStatus)
		{
			m_initialRevoluteJointStatus = initialRevoluteJointStatus;
		}

		void setMotorSpeed(double motorSpeed)
		{
			m_revoluteJoint->SetMotorSpeed(static_cast<float32>(motorSpeed));
		}

		void init()
		{
			m_revoluteJoint = static_cast<RevoluteJointPtr>(m_world.lock()->CreateJoint(&m_initialRevoluteJointStatus->getRevoluteJointDefinition()));
		}

	private:

		WorldWeakPtr m_world;

		InitialRevoluteJointStatusPtr m_initialRevoluteJointStatus;

		RevoluteJointPtr m_revoluteJoint;

	};
	
	class Box2DInitialDistanceJointStatus
	{
	public:

		using DistanceJointDef = b2DistanceJointDef;
		using Box2DBodyPtr = std::shared_ptr<Box2DBody>;

		void init(const Box2DBodyPtr& bodyA, const Box2DBodyPtr& bodyB, const Vec2& anchorA, const Vec2& anchorB)
		{
			m_distanceJointDef.Initialize(bodyA->getBodyPtr(), bodyB->getBodyPtr(), Vec2ToB2Vec2(anchorA), Vec2ToB2Vec2(anchorB));
		}

		void setDampingRatio(double dampingRatio)
		{
			m_distanceJointDef.dampingRatio = static_cast<float32>(dampingRatio);
		}

		void setFrequencyHz(double frequencyHz)
		{
			m_distanceJointDef.frequencyHz = static_cast<float32>(frequencyHz);
		}

		const DistanceJointDef& getDistanceJointDefinition() const
		{
			return m_distanceJointDef;
		}

	private:

		DistanceJointDef m_distanceJointDef;
	};

	class Box2DDistanceJoint
	{
	public:
		using WorldWeakPtr = std::weak_ptr<b2World>;
		using InitialDistanceJointStatusPtr = std::shared_ptr<Box2DInitialDistanceJointStatus>;
		using DistanceJointPtr = b2DistanceJoint*;

		Box2DDistanceJoint(const WorldWeakPtr& world)
			: m_world(world)
		{

		}

		virtual ~Box2DDistanceJoint()
		{
			if (!m_world.expired())
				m_world.lock()->DestroyJoint(m_distanceJoint);
		}

		void setInitialDistanceJointStatus(const InitialDistanceJointStatusPtr& initialDistanceJointStatus)
		{
			m_initialDistanceJointStatus = initialDistanceJointStatus;
		}

		void init()
		{
			m_distanceJoint = static_cast<DistanceJointPtr>(m_world.lock()->CreateJoint(&m_initialDistanceJointStatus->getDistanceJointDefinition()));
		}

		void draw() const
		{
			Line(B2Vec2ToVec2(m_distanceJoint->GetAnchorA()), B2Vec2ToVec2(m_distanceJoint->GetAnchorB())).draw(0.1);
		}

	private:

		WorldWeakPtr m_world;

		InitialDistanceJointStatusPtr m_initialDistanceJointStatus;

		DistanceJointPtr m_distanceJoint;
	};

	class Box2DManager
	{
	public:

		using WorldPtr = std::shared_ptr<b2World>;

		Box2DManager()
		{
			b2Vec2 gravity(0.0f, -10.0f);

			m_world = std::make_shared<b2World>(gravity);
		}

		virtual ~Box2DManager()
		{

		}

		void update()
		{
			m_world->Step(m_timeStep, m_velocityIterations, m_positionIterations);
		}

		const WorldPtr& getWorld() const
		{
			return m_world;
		}

	private:

		WorldPtr m_world;

		float32 m_timeStep = 1.0f / 60.0f;

		int32 m_velocityIterations = 6;

		int32 m_positionIterations = 2;

	};
}
