"""create missing tables

Revision ID: 4beab1917973
Revises: 5a45129dcc34
Create Date: 2025-12-06 12:35:32.086752

"""
from alembic import op
import sqlalchemy as sa


# revision identifiers, used by Alembic.
revision = '4beab1917973'
down_revision = '5a45129dcc34'
branch_labels = None
depends_on = None


def upgrade() -> None:
    # Create targetlanguage enum type (hardwareprofile already exists)
    targetlanguage_enum = sa.Enum('roman_urdu', 'formal_urdu', name='targetlanguage')
    targetlanguage_enum.create(op.get_bind(), checkfirst=True)

    # Create chat_logs table
    op.create_table(
        'chat_logs',
        sa.Column('id', sa.UUID(), nullable=False),
        sa.Column('user_id', sa.UUID(), nullable=False),
        sa.Column('query_text', sa.Text(), nullable=False),
        sa.Column('response_text', sa.Text(), nullable=False),
        sa.Column('cited_chapters', sa.dialects.postgresql.JSONB(), nullable=False),
        sa.Column('skills_loaded', sa.dialects.postgresql.JSONB(), nullable=False),
        sa.Column('sanitized_input', sa.Text(), nullable=False),
        sa.Column('created_at', sa.DateTime(), nullable=False),
        sa.ForeignKeyConstraint(['user_id'], ['users.id'], ),
        sa.PrimaryKeyConstraint('id')
    )
    op.create_index(op.f('ix_chat_logs_user_id'), 'chat_logs', ['user_id'])
    op.create_index(op.f('ix_chat_logs_created_at'), 'chat_logs', ['created_at'])

    # Create personalized_content table
    # Note: hardwareprofile enum type already exists from previous migration
    op.create_table(
        'personalized_content',
        sa.Column('id', sa.UUID(), nullable=False),
        sa.Column('user_id', sa.UUID(), nullable=False),
        sa.Column('chapter_id', sa.String(), nullable=False),
        sa.Column('hardware_profile', sa.Enum('rtx_4090', 'jetson_orin_nano', name='hardwareprofile', create_type=False), nullable=False),
        sa.Column('personalized_mdx', sa.Text(), nullable=False),
        sa.Column('created_at', sa.DateTime(), nullable=False),
        sa.ForeignKeyConstraint(['user_id'], ['users.id'], ),
        sa.PrimaryKeyConstraint('id')
    )
    op.create_index(op.f('ix_personalized_content_user_id'), 'personalized_content', ['user_id'])
    op.create_index(op.f('ix_personalized_content_chapter_id'), 'personalized_content', ['chapter_id'])
    op.create_index(op.f('ix_personalized_content_created_at'), 'personalized_content', ['created_at'])

    # Create translated_content table
    op.create_table(
        'translated_content',
        sa.Column('id', sa.UUID(), nullable=False),
        sa.Column('user_id', sa.UUID(), nullable=False),
        sa.Column('chapter_id', sa.String(), nullable=False),
        sa.Column('target_language', sa.Enum('roman_urdu', 'formal_urdu', name='targetlanguage', create_type=False), nullable=False),
        sa.Column('translated_mdx', sa.Text(), nullable=False),
        sa.Column('created_at', sa.DateTime(), nullable=False),
        sa.ForeignKeyConstraint(['user_id'], ['users.id'], ),
        sa.PrimaryKeyConstraint('id')
    )
    op.create_index(op.f('ix_translated_content_user_id'), 'translated_content', ['user_id'])
    op.create_index(op.f('ix_translated_content_chapter_id'), 'translated_content', ['chapter_id'])
    op.create_index(op.f('ix_translated_content_created_at'), 'translated_content', ['created_at'])

    # Create audit_logs table
    op.create_table(
        'audit_logs',
        sa.Column('id', sa.UUID(), nullable=False),
        sa.Column('user_id', sa.UUID(), nullable=True),
        sa.Column('event_type', sa.String(), nullable=False),
        sa.Column('event_details', sa.dialects.postgresql.JSONB(), nullable=False),
        sa.Column('ip_address', sa.String(), nullable=True),
        sa.Column('created_at', sa.DateTime(), nullable=False),
        sa.ForeignKeyConstraint(['user_id'], ['users.id'], ),
        sa.PrimaryKeyConstraint('id')
    )
    op.create_index(op.f('ix_audit_logs_user_id'), 'audit_logs', ['user_id'])
    op.create_index(op.f('ix_audit_logs_event_type'), 'audit_logs', ['event_type'])
    op.create_index(op.f('ix_audit_logs_created_at'), 'audit_logs', ['created_at'])


def downgrade() -> None:
    op.drop_index(op.f('ix_audit_logs_created_at'), table_name='audit_logs')
    op.drop_index(op.f('ix_audit_logs_event_type'), table_name='audit_logs')
    op.drop_index(op.f('ix_audit_logs_user_id'), table_name='audit_logs')
    op.drop_table('audit_logs')

    op.drop_index(op.f('ix_translated_content_created_at'), table_name='translated_content')
    op.drop_index(op.f('ix_translated_content_chapter_id'), table_name='translated_content')
    op.drop_index(op.f('ix_translated_content_user_id'), table_name='translated_content')
    op.drop_table('translated_content')
    op.execute('DROP TYPE targetlanguage')

    op.drop_index(op.f('ix_personalized_content_created_at'), table_name='personalized_content')
    op.drop_index(op.f('ix_personalized_content_chapter_id'), table_name='personalized_content')
    op.drop_index(op.f('ix_personalized_content_user_id'), table_name='personalized_content')
    op.drop_table('personalized_content')
    op.execute('DROP TYPE hardwareprofile')

    op.drop_index(op.f('ix_chat_logs_created_at'), table_name='chat_logs')
    op.drop_index(op.f('ix_chat_logs_user_id'), table_name='chat_logs')
    op.drop_table('chat_logs')
